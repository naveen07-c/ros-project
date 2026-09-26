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

// stand_hold (M5, AT-05): sends a joint-space trajectory goal to the
// joint_trajectory_controller to move the robot from its straight spawn
// pose into a symmetric standing crouch (configurable, 0 = pure hold),
// then keeps the goal active so the controller holds the pose.
//
// Joint-angle convention (see urdf/leg.xacro): pitch about +Y.
// For a flat-footed crouch of depth c:
//   hip_pitch = -c (flexion), knee = +2c (flexion), ankle_pitch = -c
// so the soles stay flat and the pelvis drops straight down.
// All values are ASSUMPTIONS per PROJECT_REQUIREMENTS.md (§12).

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory =
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

using namespace std::chrono_literals;  // NOLINT

static constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

class StandHold : public rclcpp::Node
{
public:
  StandHold()
  : Node("stand_hold")
  {
    crouch_deg_ = declare_parameter<double>("crouch_deg", 15.0);
    transition_time_ = declare_parameter<double>("transition_time", 2.0);
    action_name_ = declare_parameter<std::string>(
      "action_name", "joint_trajectory_controller/follow_joint_trajectory");

    joints_ = {
      "left_hip_roll", "left_hip_pitch", "left_knee",
      "left_ankle_pitch", "left_ankle_roll",
      "right_hip_roll", "right_hip_pitch", "right_knee",
      "right_ankle_pitch", "right_ankle_roll"};

    // Follow simulation time so logging and any future time-based logic
    // stay consistent with Gazebo (/clock).
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&StandHold::joint_state_cb, this, std::placeholders::_1));

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, action_name_);

    timer_ = create_wall_timer(
      500ms, std::bind(&StandHold::try_send_goal, this));

    RCLCPP_INFO(
      get_logger(),
      "stand_hold: crouch_deg=%.1f, transition_time=%.2f s, waiting for "
      "/joint_states and action server '%s'",
      crouch_deg_, transition_time_, action_name_.c_str());
  }

  ~StandHold() override
  {
    // Emergency-safe stop (PROJECT_REQUIREMENTS §12): cancel the goal so the
    // controller stops streaming commands.
    if (action_client_ && goal_handle_) {
      action_client_->async_cancel_goal(goal_handle_);
    }
  }

private:
  void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::vector<double> positions(joints_.size(), 0.0);
    bool all_found = true;
    for (size_t i = 0; i < joints_.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), joints_[i]);
      if (it == msg->name.end() ||
        static_cast<size_t>(std::distance(msg->name.begin(), it)) >=
        msg->position.size())
      {
        all_found = false;
        break;
      }
      positions[i] = msg->position[std::distance(msg->name.begin(), it)];
    }
    if (all_found) {
      latest_positions_ = positions;
    }
  }

  std::vector<double> crouch_targets() const
  {
    std::vector<double> targets(joints_.size(), 0.0);
    const double c = crouch_deg_ * kDeg2Rad;
    // indices per joints_ order above
    targets[1] = -c;        // left_hip_pitch
    targets[2] = 2.0 * c;   // left_knee
    targets[3] = -c;        // left_ankle_pitch
    targets[6] = -c;        // right_hip_pitch
    targets[7] = 2.0 * c;   // right_knee
    targets[8] = -c;        // right_ankle_pitch
    return targets;
  }

  void try_send_goal()
  {
    if (goal_sent_ || latest_positions_.empty()) {
      return;
    }
    if (!action_client_->action_server_is_ready()) {
      return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joints_;

    // First point = current measured pose: JTC accepts the goal smoothly
    // (start matches state within trajectory tolerance).
    trajectory_msgs::msg::JointTrajectoryPoint start;
    start.positions = latest_positions_;
    start.time_from_start = rclcpp::Duration::from_seconds(0.0);
    goal.trajectory.points.push_back(start);

    trajectory_msgs::msg::JointTrajectoryPoint target;
    target.positions = crouch_targets();
    target.time_from_start = rclcpp::Duration::from_seconds(transition_time_);
    goal.trajectory.points.push_back(target);

    auto send_options =
      rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

    send_options.goal_response_callback =
      [this](const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "stand_hold: goal REJECTED by controller");
          goal_sent_ = false;  // allow retry on next timer tick
        } else {
          goal_handle_ = goal_handle;
          RCLCPP_INFO(
            get_logger(),
            "stand_hold: goal accepted - executing %.1f s crouch-in, then the "
            "controller holds the pose", transition_time_);
        }
      };

    send_options.result_callback =
      [this](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
        goal_handle_ = nullptr;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(
              get_logger(), "stand_hold: goal SUCCEEDED (pose held by JTC)");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(
              get_logger(), "stand_hold: goal ABORTED - check tracking "
              "constraints / robot stability");
            goal_sent_ = false;  // allow retry
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "stand_hold: goal CANCELED");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "stand_hold: unknown result code");
            goal_sent_ = false;
            break;
        }
      };

    goal_sent_ = true;
    action_client_->async_send_goal(goal, send_options);
  }

  double crouch_deg_;
  double transition_time_;
  std::string action_name_;
  std::vector<std::string> joints_;
  std::vector<double> latest_positions_;
  bool goal_sent_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  GoalHandleFollowJointTrajectory::SharedPtr goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StandHold>());
  rclcpp::shutdown();
  return 0;
}
