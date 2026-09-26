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

// gait_odometry (M6, M8): gait-progress / base-pose estimate for a legged
// robot (the reinterpreted PDF "odometry" milestone - there are no
// wheels, so wheel odometry does not apply; PROJECT_REQUIREMENTS C2).
//
// Method: stance-foot kinematic odometry. A foot is "in stance" when its
// contact sensor (M8, /<side>_foot_contact) reports contacts above a
// force threshold; while both or neither report, the tracked point is
// the mean of both ankles. Its pose is reconstructed from joint states
// via the fixed leg kinematics (hip->knee->ankle chain, pitch about +Y)
// and tracked across the stance phase. The displacement of the tracked
// point is applied (negated, pitch-rotated) to the odom->base_link
// transform every cycle, so pelvis motion is the integral of the
// planted-foot motion - identical in structure to leg odometry.
//
// M8 changes (this file):
//   - Stance detection now comes from the foot CONTACT SENSORS, not the
//     gait generator's /gait_support topic (that topic is still consumed
//     as a fallback if the contacts have never been seen - e.g. running
//     the node against an older world without the sensor systems).
//   - Base pitch is fused from the IMU (/imu) with complementary
//     filtering (accelerometer tilt, low-passed) instead of the mean leg
//     sagittal angles, per AT-08 "IMU ... sensible data, frames".
//
// Frames:
//   odom   - world-fixed at spawn, z = base spawn height, x forward
//   base_link - pelvis frame (URDF root); IMU frame == base_link
//   *_foot - foot frames from robot_state_publisher TF
//
// Documented gap (closed in M8): before the contact sensors existed,
// /gait_support was the stance source of truth.

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;  // NOLINT

static constexpr double kDeg = 3.14159265358979323846 / 180.0;

class GaitOdometry : public rclcpp::Node
{
public:
  GaitOdometry()
  : Node("gait_odometry")
  {
    thigh_ = declare_parameter<double>("thigh_length", 0.20);
    shin_ = declare_parameter<double>("shin_length", 0.20);
    hip_y_ = declare_parameter<double>("hip_y", 0.10);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    left_foot_frame_ = declare_parameter<std::string>("left_foot_frame", "left_foot");
    right_foot_frame_ = declare_parameter<std::string>(
      "right_foot_frame", "right_foot");

    // M8 parameters: contact threshold and IMU fusion tuning (ASSUMPTIONS,
    // PROJECT_REQUIREMENTS section 12).
    contact_force_min_ = declare_parameter<double>("contact_force_min", 1.0);
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    left_contact_topic_ =
      declare_parameter<std::string>("left_contact_topic", "/left_foot_contact");
    right_contact_topic_ =
      declare_parameter<std::string>("right_contact_topic", "/right_foot_contact");
    pitch_tau_ = declare_parameter<double>("pitch_filter_tau", 0.5);

    set_parameter(rclcpp::Parameter("use_sim_time", true));

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&GaitOdometry::joint_state_cb, this, std::placeholders::_1));
    support_sub_ = create_subscription<std_msgs::msg::String>(
      "/gait_support", 10,
      std::bind(&GaitOdometry::support_cb, this, std::placeholders::_1));

    // M8: foot contacts + IMU.
    left_contact_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      left_contact_topic_, 10,
      std::bind(&GaitOdometry::left_contact_cb, this, std::placeholders::_1));
    right_contact_sub_ = create_subscription<ros_gz_interfaces::msg::Contacts>(
      right_contact_topic_, 10,
      std::bind(&GaitOdometry::right_contact_cb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, 10,
      std::bind(&GaitOdometry::imu_cb, this, std::placeholders::_1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    stance_pub_ = create_publisher<std_msgs::msg::String>("/stance_foot", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(),
      "gait_odometry: stance-foot kinematic odometry on /odom + TF "
      "%s -> %s (stance source: FOOT CONTACTS on %s / %s, force > %.1f N; "
      "/gait_support fallback only; pitch from %s)",
      odom_frame_.c_str(), base_frame_.c_str(), left_contact_topic_.c_str(),
      right_contact_topic_.c_str(), contact_force_min_, imu_topic_.c_str());
  }

private:
  struct LegPose
  {
    double x = 0.0;         // ankle x relative to pelvis (+forward)
    double z = 0.0;         // ankle z below the pelvis (positive down)
    double leg_pitch = 0.0; // mean sagittal leg angle
  };

  // ---- M8: contact sensor handling ----
  //
  // A gz.msgs.Contacts message carries one entry per contacting pair
  // (collision names + per-point wrenches). The URDF only declares one
  // collision per foot, so any message with a sufficiently loaded
  // contact counts as "foot on ground". Note that Gazebo publishes
  // Contacts at a nonzero rate only while a contact EXISTS; a foot in
  // the air stays silent (that silence IS the swing signal).
  //
  // Jazzy ros_gz_interfaces/Contact: wrenches is a JointWrench[] with
  // body_1_wrench / body_2_wrench (equal-opposite pair at the contact
  // joint); the body_1 force magnitude is the contact normal load.
  double total_contact_force(
    const ros_gz_interfaces::msg::Contacts & m) const
  {
    double f = 0.0;
    for (const auto & c : m.contacts) {
      for (const auto & jw : c.wrenches) {
        f += std::sqrt(
          jw.body_1_wrench.force.x * jw.body_1_wrench.force.x +
          jw.body_1_wrench.force.y * jw.body_1_wrench.force.y +
          jw.body_1_wrench.force.z * jw.body_1_wrench.force.z);
      }
    }
    // Presence fallback: some entries may carry no wrench data; a
    // contact without wrenches still proves sole-to-ground proximity.
    if (!m.contacts.empty() && f < 1e-9) {
      f = contact_force_min_;
    }
    return f;
  }

  void left_contact_cb(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    left_contact_seen_ = true;
    if (total_contact_force(*msg) >= contact_force_min_) {
      last_left_contact_ = now();
    }
  }

  void right_contact_cb(const ros_gz_interfaces::msg::Contacts::SharedPtr msg)
  {
    right_contact_seen_ = true;
    if (total_contact_force(*msg) >= contact_force_min_) {
      last_right_contact_ = now();
    }
  }

  // Stance booleans with a hangover window: contact messages stop while
  // the foot is in the air, so "recently reported" has to time out. The
  // window (200 ms) is longer than the ~10 ms joint-state cycle but far
  // shorter than a step segment (>= 100 ms at the M6 timings); the
  // physics contact persists across the whole stance phase, refreshing
  // last_*_contact_ at the 200 Hz sensor rate.
  bool left_in_stance() const
  {
    return (now() - last_left_contact_).seconds() < stance_window_;
  }
  bool right_in_stance() const
  {
    return (now() - last_right_contact_).seconds() < stance_window_;
  }

  // ---- IMU: complementary-filtered base pitch ----

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    imu_seen_ = true;
    // Accelerometer tilt: gravity measured in body frame gives the pitch
    // of the body relative to the world (IMU frame == base_link, sensor
    // poses are identity in sensors.xacro).
    const double ax = msg->linear_acceleration.x;
    const double az = msg->linear_acceleration.z;
    if (std::abs(ax) < 1e-6 && std::abs(az) < 1e-6) {
      return;  // degenerate sample (free fall / uninitialized)
    }
    const double accel_pitch = std::atan2(-ax, az);
    // Complementary filter: accelerometer corrects the slow drift of the
    // integrated gyro.
    const double alpha = std::clamp(pitch_tau_ / (pitch_tau_ + dt_est_), 0.0, 1.0);
    imu_pitch_ = alpha * (imu_pitch_ + msg->angular_velocity.y * dt_est_) +
      (1.0 - alpha) * accel_pitch;
    last_imu_ = now();
  }

  bool imu_pitch_fresh() const
  {
    return imu_seen_ && (now() - last_imu_).seconds() < 0.5;
  }

  void support_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string s = msg->data;
    std::string norm;
    if (s == "left" || s == "right") {
      norm = s;
    }
    if (!left_contact_seen_ && !right_contact_seen_) {
      // Fallback only: no contact sensors have ever published (old world
      // without the M8 sensor systems). Track the generator's stance.
      if (norm != support_) {
        support_ = norm;
        tracked_ref_valid_ = false;
      }
    }
    // With contacts available this topic is ignored (logged once below).
  }

  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::array<double, 2> hip_roll{0.0, 0.0};
    std::array<double, 2> hip_pitch{0.0, 0.0};
    std::array<double, 2> knee{0.0, 0.0};
    std::array<double, 2> ankle_pitch{0.0, 0.0};
    bool ok = true;
    for (int side = 0; side < 2 && ok; ++side) {
      const std::string p = side == 0 ? "left_" : "right_";
      ok = lookup(*msg, p + "hip_roll", hip_roll[side]) &&
        lookup(*msg, p + "hip_pitch", hip_pitch[side]) &&
        lookup(*msg, p + "knee", knee[side]) &&
        lookup(*msg, p + "ankle_pitch", ankle_pitch[side]);
    }
    if (!ok) {
      return;
    }
    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }
    update(hip_pitch, knee, ankle_pitch, stamp);
  }

  static bool lookup(
    const sensor_msgs::msg::JointState & msg, const std::string & name,
    double & out)
  {
    for (size_t i = 0; i < msg.name.size(); ++i) {
      if (msg.name[i] == name && i < msg.position.size()) {
        out = msg.position[i];
        return true;
      }
    }
    return false;
  }

  // Fixed-link forward kinematics of one leg (leg.xacro convention):
  // pitch joints about +Y, chain base_link -> hip -> thigh(l1) ->
  // shin(l2) -> ankle. Returns the ankle position in the pelvis frame
  // (+x forward, +z up) and the mean sagittal leg pitch.
  LegPose leg_fk(
    double hip_pitch, double knee, double ankle_pitch, int side) const
  {
    const double l1 = thigh_;
    const double l2 = shin_;
    // Joint axes are +Y; a positive rotation about +Y swings the leg's
    // -Z (down) direction toward -X, so dx uses -sin, dz uses -cos.
    const double a1 = hip_pitch;
    const double a2 = a1 + knee;
    const double x = -l1 * std::sin(a1) - l2 * std::sin(a2);
    const double z = -l1 * std::cos(a1) - l2 * std::cos(a2);
    const double pitch = (a1 + a2 + (a2 + ankle_pitch)) / 3.0;
    LegPose p;
    p.x = x;
    p.z = z;
    p.leg_pitch = pitch;
    // hip roll shifts the ankle laterally by the hip offset only (both
    // sides measured from their own hip) - no x/z coupling for odometry.
    (void)side;
    return p;
  }

  void update(
    const std::array<double, 2> & hip_pitch,
    const std::array<double, 2> & knee,
    const std::array<double, 2> & ankle_pitch,
    const rclcpp::Time & stamp)
  {
    // Integrate BOTH legs each cycle; the stance-foot rule selects the
    // tracked planted point (leg-odometry structure).
    std::array<LegPose, 2> lp{
      leg_fk(hip_pitch[0], knee[0], ankle_pitch[0], 0),
      leg_fk(hip_pitch[1], knee[1], ankle_pitch[1], 1)};

    // Stance selection: contact sensors first (M8), generator topic only
    // as a fallback while no contact sensor has ever published.
    if (left_contact_seen_ || right_contact_seen_) {
      const bool l = left_in_stance();
      const bool r = right_in_stance();
      const std::string contact_support =
        l && r ? "" : (l ? "left" : (r ? "right" : ""));
      if (contact_support != support_) {
        // Tracked point switches reference: re-seed (one ~10 ms cycle of
        // displacement at the transition is dropped, not integrated).
        support_ = contact_support;
        tracked_ref_valid_ = false;
      }
      if (support_fallback_) {
        support_fallback_ = false;
        RCLCPP_INFO(
          get_logger(),
          "gait_odometry: contact sensors online - stance now sourced from "
          "foot contacts (was /gait_support)");
      }
    }

    // Tracked planted point: the stance ankle in single support; the
    // mean of both ankles in double support (both feet are planted, so
    // the mean is the geometric compromise; no slip is modeled). With
    // no contact at all (transient at liftoff), keep the last rule by
    // treating both as planted.
    double tx, tz, tpitch;
    if (support_ == "left") {
      tx = lp[0].x;
      tz = lp[0].z;
      tpitch = lp[0].leg_pitch;
    } else if (support_ == "right") {
      tx = lp[1].x;
      tz = lp[1].z;
      tpitch = lp[1].leg_pitch;
    } else {
      tx = 0.5 * (lp[0].x + lp[1].x);
      tz = 0.5 * (lp[0].z + lp[1].z);
      tpitch = 0.5 * (lp[0].leg_pitch + lp[1].leg_pitch);
    }

    // Base pitch: IMU when fresh (M8), mean leg pitch otherwise.
    double fused_pitch;
    if (imu_pitch_fresh()) {
      fused_pitch = imu_pitch_;
    } else {
      fused_pitch = tpitch;
    }

    if (!tracked_ref_valid_) {
      // (re)seed the planted-point reference after a support change
      prev_x_ = tx;
      prev_z_ = tz;
      pitch_ = fused_pitch;
      tracked_ref_valid_ = true;
      publish(stamp);
      return;
    }

    // Displacement of the planted point since the last cycle, applied
    // negatively to the base: if the planted feet move +x relative to
    // the pelvis, the pelvis moved -x relative to the world.
    const double dx = -(tx - prev_x_);
    const double dz = -(tz - prev_z_);
    prev_x_ = tx;
    prev_z_ = tz;

    const double c = std::cos(pitch_);
    const double s = std::sin(pitch_);
    base_x_ += c * dx + s * dz;
    base_z_ += -s * dx + c * dz;
    pitch_ = fused_pitch;

    publish(stamp);
  }

  void publish(const rclcpp::Time & stamp)
  {
    nav_msgs::msg::Odometry odo;
    odo.header.stamp = stamp;
    odo.header.frame_id = odom_frame_;
    odo.child_frame_id = base_frame_;
    odo.pose.pose.position.x = base_x_;
    odo.pose.pose.position.y = 0.0;
    odo.pose.pose.position.z = base_z_;

    tf2::Quaternion q;
    q.setRPY(0.0, pitch_, 0.0);
    odo.pose.pose.orientation.x = q.x();
    odo.pose.pose.orientation.y = q.y();
    odo.pose.pose.orientation.z = q.z();
    odo.pose.pose.orientation.w = q.w();

    // Pose-covariance diagnostics only (simulation): x/z pose is exact
    // kinematics; everything else is unobserved (large sigma).
    odo.pose.covariance[0] = 1e-6;    // x
    odo.pose.covariance[7] = 1e9;     // y (unobserved in the sagittal plane)
    odo.pose.covariance[14] = 1e-6;   // z
    odo.pose.covariance[21] = 1e9;    // roll
    odo.pose.covariance[28] = 1e-4;   // pitch (IMU-fused estimate)
    odo.pose.covariance[35] = 1e9;    // yaw

    odom_pub_->publish(odo);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = base_x_;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = base_z_;
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf);

    // Diagnostics: current stance ruling (contacts or fallback). Published
    // on CHANGE (for offline transition analysis) plus a 1 s heartbeat.
    std::string stance_str;
    if (support_fallback_) {
      stance_str = support_.empty() ? "none(fallback)" :
        support_ + "(fallback)";
    } else {
      stance_str = support_.empty() ? "none" : support_;
    }
    if (stance_str != last_stance_str_ ||
      now() - last_diag_ > rclcpp::Duration::from_seconds(1.0))
    {
      last_diag_ = now();
      last_stance_str_ = stance_str;
      std_msgs::msg::String m;
      m.data = stance_str;
      stance_pub_->publish(m);
    }
  }

  double thigh_, shin_, hip_y_;
  std::string odom_frame_, base_frame_, left_foot_frame_, right_foot_frame_;

  // M8 configuration
  double contact_force_min_;
  std::string imu_topic_, left_contact_topic_, right_contact_topic_;
  double pitch_tau_;
  double dt_est_{0.01};   // joint-state period estimate (100 Hz)
  double stance_window_{0.2};  // contact hangover window (s)

  std::string support_;          // "", "left", "right" (tracked stance)
  bool left_contact_seen_{false};
  bool right_contact_seen_{false};
  bool support_fallback_{true};
  rclcpp::Time last_left_contact_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_right_contact_{0, 0, RCL_ROS_TIME};

  bool imu_seen_{false};
  double imu_pitch_{0.0};
  rclcpp::Time last_imu_{0, 0, RCL_ROS_TIME};

  double prev_x_{0.0};           // tracked stance ankle x (pelvis frame)
  double prev_z_{0.0};           // tracked stance ankle z (pelvis frame)
  bool tracked_ref_valid_{false};
  double base_x_{0.0};           // integrated base pose in odom
  double base_z_{0.0};
  double pitch_{0.0};
  rclcpp::Time last_diag_{0, 0, RCL_ROS_TIME};
  std::string last_stance_str_;   // last published stance diagnostic

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr support_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr
    left_contact_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr
    right_contact_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stance_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaitOdometry>());
  rclcpp::shutdown();
  return 0;
}
