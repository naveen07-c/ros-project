// Copyright 2026 sudeep
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// contact_markers (M9, AT-09): RViz visualization of the M8 sensor data.
//
//   - Foot contact force arrows: one Arrow marker per loaded foot,
//     anchored at the sole (TF odom -> <side>_foot composed with the
//     sole offset from leg.xacro), pointing along the measured contact
//     normal with length proportional to the force magnitude. Markers
//     carry a short lifetime, so a lifted (silent) foot simply stops
//     being drawn - the same "silence = swing" semantics as the data.
//   - IMU measured-up arrow: the accelerometer's measured direction
//     (in base_link) rotated into odom by TF and drawn from the pelvis.
//     Upright + static = straight up, length 0.2 m.
//
// Rationale: no rviz IMU display plugin ships with the distro, and the
// contact forces are otherwise invisible. Markers are published on
// /contact_markers (visualization_msgs/MarkerArray).

#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Vector3.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

static constexpr double kScale = 0.004;  // m of arrow per newton

class ContactMarkers : public rclcpp::Node
{
public:
  ContactMarkers()
  : Node("contact_markers")
  {
    left_contact_topic_ =
      declare_parameter<std::string>("left_contact_topic", "/left_foot_contact");
    right_contact_topic_ =
      declare_parameter<std::string>("right_contact_topic", "/right_foot_contact");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    left_foot_frame_ = declare_parameter<std::string>("left_foot_frame", "left_foot");
    right_foot_frame_ = declare_parameter<std::string>(
      "right_foot_frame", "right_foot");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    marker_lifetime_ = declare_parameter<double>("marker_lifetime", 0.15);
    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);

    set_parameter(rclcpp::Parameter("use_sim_time", true));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/contact_markers", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    left_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      left_contact_topic_, 10,
      std::bind(&ContactMarkers::left_contact_cb, this, std::placeholders::_1));
    right_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      right_contact_topic_, 10,
      std::bind(&ContactMarkers::right_contact_cb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&ContactMarkers::imu_cb, this, std::placeholders::_1));

    // One combined MarkerArray per timer tick (~30 Hz): the callbacks
    // only store state, so a 700+ Hz contact stream does not flood RViz
    // with thousands of tiny messages.
    timer_ = create_wall_timer(
      std::chrono::milliseconds(
        static_cast<int>(1000.0 / std::max(1.0, publish_rate_))),
      std::bind(&ContactMarkers::publish_markers, this));

    RCLCPP_INFO(
      get_logger(),
      "contact_markers: foot contact arrows (%s, %s) + IMU measured-up "
      "arrow (%s) on /contact_markers at %.0f Hz",
      left_contact_topic_.c_str(), right_contact_topic_.c_str(),
      imu_topic_.c_str(), publish_rate_);
  }

private:
  struct Pose
  {
    tf2::Vector3 translation{0.0, 0.0, 0.0};
    tf2::Quaternion rotation{0.0, 0.0, 0.0, 1.0};
  };

  bool lookup_pose(const std::string & child, Pose & out)
  {
    try {
      const geometry_msgs::msg::TransformStamped t =
        tf_buffer_->lookupTransform(
        odom_frame_, child, tf2::TimePointZero);
      out.translation = tf2::Vector3{
        t.transform.translation.x, t.transform.translation.y,
        t.transform.translation.z};
      out.rotation = tf2::Quaternion{
        t.transform.rotation.x, t.transform.rotation.y,
        t.transform.rotation.z, t.transform.rotation.w};
      return true;
    } catch (const tf2::TransformException & e) {
      static rclcpp::Time last_warn{0, 0, RCL_ROS_TIME};
      const rclcpp::Time n = now();
      if ((n - last_warn).seconds() > 5.0) {
        last_warn = n;
        RCLCPP_WARN(
          get_logger(), "TF lookup %s <- %s failed: %s",
          odom_frame_.c_str(), child.c_str(), e.what());
      }
      return false;
    }
  }

  // Sole center in the foot frame (leg.xacro: box centered at
  // x=0.02, z = -(ankle_h - foot_h/2)); keep in sync with leg.xacro.
  static tf2::Vector3 sole_offset()
  {
    return tf2::Vector3{0.02, 0.0, -0.135};
  }

  tf2::Vector3 rotate(const tf2::Quaternion & q, const tf2::Vector3 & v) const
  {
    tf2::Quaternion vq{v.x(), v.y(), v.z(), 0.0};
    const tf2::Quaternion out = q * vq * q.inverse();
    return tf2::Vector3{out.x(), out.y(), out.z()};
  }

  struct FootState
  {
    double force = 0.0;
    tf2::Vector3 normal{0.0, 0.0, 1.0};
    bool fresh = false;
  };

  void contact_cb(
    const ros_gz_interfaces::msg::Contacts & msg, FootState & state)
  {
    // Aggregate the contact load and force-weighted normal.
    double force = 0.0;
    tf2::Vector3 normal_acc{0.0, 0.0, 0.0};
    for (const auto & c : msg.contacts) {
      for (const auto & jw : c.wrenches) {
        const auto & f = jw.body_1_wrench.force;
        const double m = std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
        force += m;
        if (m > 1e-6 && !c.normals.empty()) {
          normal_acc += tf2::Vector3{c.normals[0].x, c.normals[0].y,
            c.normals[0].z} * (m / m);
        }
      }
    }
    state.force = force;
    state.fresh = force >= 1.0 && normal_acc.length2() > 1e-12;
    if (state.fresh) {
      normal_acc.normalize();
      state.normal = normal_acc;
    }
  }

  void left_contact_cb(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    contact_cb(*msg, left_);
  }

  void right_contact_cb(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    contact_cb(*msg, right_);
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Vector3 a{msg->linear_acceleration.x,
      msg->linear_acceleration.y, msg->linear_acceleration.z};
    if (a.length2() < 1e-9) {
      return;
    }
    a.normalize();
    imu_up_ = a;
    imu_fresh_ = true;
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray arr;
    const rclcpp::Time stamp = now();

    // Foot force arrows (green = left, orange = right).
    const std::array<FootState *, 2> feet{&left_, &right_};
    const std::array<std::string, 2> frames{left_foot_frame_,
      right_foot_frame_};
    const std::array<std::array<float, 3>, 2> colors{
      std::array<float, 3>{0.2f, 1.0f, 0.2f},
      std::array<float, 3>{1.0f, 0.6f, 0.1f}};
    for (size_t i = 0; i < 2; ++i) {
      if (!feet[i]->fresh) {
        continue;  // lifted foot: no marker (lifetime expires the old one)
      }
      Pose foot;
      if (!lookup_pose(frames[i], foot)) {
        continue;
      }
      const tf2::Vector3 anchor =
        foot.translation + rotate(foot.rotation, sole_offset());

      visualization_msgs::msg::Marker m;
      m.header.frame_id = odom_frame_;
      m.header.stamp = stamp;
      m.ns = "foot_force";
      m.id = static_cast<int32_t>(i);
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.02;  // shaft diameter
      m.scale.y = 0.05;  // head diameter
      m.scale.z = 0.05;  // head length
      m.color.r = colors[i][0];
      m.color.g = colors[i][1];
      m.color.b = colors[i][2];
      m.color.a = 0.9f;
      m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);

      geometry_msgs::msg::Point start;
      start.x = anchor.x();
      start.y = anchor.y();
      start.z = anchor.z();
      const tf2::Vector3 tip =
        anchor + feet[i]->normal * (kScale * feet[i]->force);
      geometry_msgs::msg::Point end;
      end.x = tip.x();
      end.y = tip.y();
      end.z = tip.z();
      m.points = {start, end};
      m.colors = {m.color, m.color};
      arr.markers.push_back(m);
    }

    // IMU measured-up arrow from the pelvis.
    if (imu_fresh_) {
      Pose base;
      if (lookup_pose(base_frame_, base)) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = odom_frame_;
        m.header.stamp = stamp;
        m.ns = "imu_up";
        m.id = 2;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = 0.02;
        m.scale.y = 0.06;
        m.scale.z = 0.06;
        m.color.r = 0.1f;
        m.color.g = 0.8f;
        m.color.b = 1.0f;
        m.color.a = 0.9f;
        m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);

        geometry_msgs::msg::Point start;
        start.x = base.translation.x();
        start.y = base.translation.y();
        start.z = base.translation.z();
        const tf2::Vector3 tip = base.translation + rotate(
          base.rotation, imu_up_) * 0.2;
        geometry_msgs::msg::Point end;
        end.x = tip.x();
        end.y = tip.y();
        end.z = tip.z();
        m.points = {start, end};
        m.colors = {m.color, m.color};
        arr.markers.push_back(m);
      }
    }

    if (!arr.markers.empty()) {
      marker_pub_->publish(arr);
    }
  }

  std::string left_contact_topic_, right_contact_topic_, imu_topic_;
  std::string odom_frame_, left_foot_frame_, right_foot_frame_, base_frame_;
  double marker_lifetime_;
  double publish_rate_;

  FootState left_;
  FootState right_;
  tf2::Vector3 imu_up_{0.0, 0.0, 1.0};
  bool imu_fresh_{false};

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr left_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ContactMarkers>());
  rclcpp::shutdown();
  return 0;
}
