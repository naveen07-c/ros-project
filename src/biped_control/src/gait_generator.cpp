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

// gait_generator (M6, AT-06/AT-10): scripted open-loop quasi-static gait.
//
// Replaces the PDF's /cmd_vel wheel interface with /gait_command
// (std_msgs/Bool: true = walk, false = stop) per conflict C2 of
// PROJECT_REQUIREMENTS.md.
//
// Gait plan (all in the sagittal plane, flat-sole IK):
//   Each step is a sequence of interpolated double/single-support poses.
//   The pelvis first shifts forward onto the (front) support foot while
//   both feet are still down, the back foot then lifts, swings ahead and
//   plants, and a final double-support settle advances the pelvis to the
//   midpoint of the new feet positions. Planted feet are never commanded
//   to translate, so no sole sliding is required; the lateral weight
//   shift (hip roll + compensating ankle roll on BOTH legs) keeps the
//   soles flat while unloading the swing foot. The first step starts
//   from feet-together and is a half step.
//
// Joint-limit guards clamp every target to the URDF limits and count the
// clamp events (PROJECT_REQUIREMENTS section 12). Emergency stop: on a
// JTC abort, a watchdog timeout, or 'false' the node freezes the current
// measured pose as a zero-velocity trajectory / cancels the goal.
//
// Height invariants (gait retune, AT-06/AT-11): in single support the
// pitch chains span d1 = sqrt(hip_y^2 + z0^2) so the rolled pelvis sits
// at the FULL stand height z0 exactly over the support foot; in double
// support both legs span z0 with the pelvis level at the feet midpoint.
// The pelvis height is therefore constant through the whole cycle - the
// only vertical motion is the swing lift, which never loads the support.
//
// Gait parameters are ASSUMPTIONS (sections 5/12) and live in the
// parameter set below plus walk.launch.py.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJointTrajectory =
  rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

using namespace std::chrono_literals;  // NOLINT

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kDeg = kPi / 180.0;

namespace
{

// Latched state; also the payload string of /gait_status.
enum class GaitState
{
  IDLE,     // standing: the JTC holds the last commanded pose
  WALKING,  // stepping through the segment queue
  STOPPING, // stop requested: land the swing foot, settle, then idle
  ESTOP     // frozen pose (safety abort)
};

const char * state_name(GaitState s)
{
  switch (s) {
    case GaitState::IDLE: return "IDLE";
    case GaitState::WALKING: return "WALKING";
    case GaitState::STOPPING: return "STOPPING";
    case GaitState::ESTOP: return "ESTOP";
  }
  return "UNKNOWN";
}

}  // namespace

class GaitGenerator : public rclcpp::Node
{
public:
  GaitGenerator()
  : Node("gait_generator")
  {
    // ---- geometry (must match urdf/leg.xacro, ASSUMPTIONS section 3) ----
    thigh_ = declare_parameter<double>("thigh_length", 0.20);
    shin_ = declare_parameter<double>("shin_length", 0.20);
    leg_len_ = thigh_ + shin_;

    // ---- gait parameters (ASSUMPTIONS, section 5) ----
    step_length_ = declare_parameter<double>("step_length", 0.06);
    step_period_ = declare_parameter<double>("step_period", 2.0);
    approach_time_ = declare_parameter<double>("approach_time", 1.5);
    swing_lift_ = declare_parameter<double>("swing_lift", 0.03);
    // hip_y must match the hip separation in biped.urdf.xacro: the
    // single-support roll shift is dictated by quasi-static balance (the
    // pelvis must pass over the support foot), not a free tuning knob.
    hip_y_ = declare_parameter<double>("hip_y", 0.10);
    // Lateral static margin: the pelvis rests this far INBOARD of the
    // support ankle in single support (roll = asin(shift/d1) with
    // span d1 = sqrt(shift^2 + z0^2)). Exactly-over-the-foot (0) is an
    // inverted pendulum with zero margin - a degree of tracking sag
    // topples it (AT-11 diagnosis).
    support_overshoot_ = declare_parameter<double>("support_overshoot", 0.02);
    support_overshoot_ =
      std::clamp(support_overshoot_, 0.0, 0.5 * hip_y_);
    // Partial lateral weight shift: the pelvis stops support_shift
    // INBOARD of the support ankle instead of rolling fully over it
    // (1.5 cm is 1/3 of the 4.5 cm sole half-width - the CoM stays
    // inside the foot polygon with margin, at a chain span barely
    // longer than z0, keeping the crouch shallow and feasible).
    support_shift_ = declare_parameter<double>("support_shift", 0.015);
    support_shift_ =
      std::clamp(support_shift_, 0.0, support_overshoot_);
    // Sagittal static margin: in single support the pelvis rests this
    // far BEHIND the support ankle (support hip_x = +m, swing sweep
    // L+m). Ground-truth gz pose (AT-11): with hip_x_sup = 0 the robot
    // pitched forward over the support foot during SWING (reaction to
    // the accelerating swing mass) - roll stayed 0, z fell 0.60 ->
    // 0.14 m. The SETTLE (both feet planted, polygon interior) does
    // the forward advance instead.
    sagittal_margin_ = declare_parameter<double>("sagittal_margin", 0.02);
    // Negative = pelvis AHEAD of the support ankle (counters the
    // backward compliance lean; the sole polygon gives 8 cm forward
    // margin at the ankle, so -0.05 is still well inside it).
    sagittal_margin_ =
      std::clamp(sagittal_margin_, -0.05, 0.5 * step_length_);
    crouch_deg_ = declare_parameter<double>("crouch_deg", 20.0);

    // segment duration fractions of step_period_ (sum = 1)
    // AT-11 diagnosis: the body-over-foot inverted pendulum has period
    // T = 2*pi*sqrt(h/g) ~= 1.55 s at h = 0.6 m. A weight shift faster
    // than ~2T injects a lateral impulse the quasi-static design cannot
    // absorb - the pelvis overshoots and every step pumps the mode
    // (resonant rocking, |a| growing step by step). frac_shift 0.30 at
    // step_period 10 s = 3 s shifts (~2T): genuinely quasi-static.
    frac_ = {
      declare_parameter<double>("frac_shift", 0.30),   // steady: pelvis onto support
      declare_parameter<double>("frac_lift", 0.10),    // swing knee bends up
      declare_parameter<double>("frac_swing", 0.30),   // swing foot moves ahead
      declare_parameter<double>("frac_plant", 0.10),   // swing foot lowers
      declare_parameter<double>("frac_settle", 0.20)};  // DS: pelvis to midpoint

    // ---- guards (section 12) ----
    max_step_length_ = declare_parameter<double>("max_step_length", 0.12);
    min_step_length_ = declare_parameter<double>("min_step_length", 0.02);
    min_step_period_ = declare_parameter<double>("min_step_period", 1.0);
    start_first_left_ = declare_parameter<bool>("start_with_left", true);
    walk_distance_limit_ = declare_parameter<double>("walk_distance_limit", 0.0);
    // Fraction of the measured pelvis attitude compensated into the
    // ankle commands (sole levelling, see try_send). Default 0: the
    // A/B campaign (AT-11 runs C-E) measured full-gain swing-sole
    // levelling as DESTABILIZING (az flip-flop +/-20 at the first
    // shift) while the open-loop config survives the whole walk.
    // Parameter kept at 0 for future experiments.
    sole_level_gain_ = declare_parameter<double>("sole_level_gain", 0.0);
    // ILC command-space integrator gain (0 disables learning).
    // Learns the gravity-sag offsets at the settled DS boundary and
    // rides them through the step. Run D showed UNBOUNDED accumulation
    // aborts the fast SWING once bias exceeds the trajectory
    // tolerance; ilc_freeze_after_steps bounds it - the per-cycle
    // geometry repeats, so a few updates capture the quasi-constant
    // sag.
    ilc_gain_ = declare_parameter<double>("ilc_gain", 0.0);
    ilc_freeze_after_ = declare_parameter<int>("ilc_freeze_after_steps", 2);
    // Static hip-pitch feedforward: the ~15 N*m gravity moment at the
    // hips sags the pelvis ~0.13 rad BEHIND the commanded pose (the
    // treadmill mechanism). A constant offset on both hip pitches
    // pre-compensates it deterministically - no feedback loop (the
    // ILC variants all measured destabilizing, AT-11 runs D-H/L).
    hip_sag_ff_ = declare_parameter<double>("hip_sag_ff", 0.0);
    // Sag-compensating over-advance: the pelvis is commanded this far
    // AHEAD of the feet midpoint at every SETTLE end (the offset is
    // carried through the next SHIFT start), so the measured per-cycle
    // sag recoil lands the pelvis ON the midpoint instead of behind it
    // (AJ: progress decelerating at ~0.03-0.05 m/cycle deficit).
    // Pure kinematic target offset - no feedback loop (the ILC and
    // FF variants all measured destabilizing; this only moves WHERE
    // the settle target sits, never adds dynamics).
    settle_boost_ = declare_parameter<double>("settle_boost", 0.0);
    settle_boost_ = std::clamp(settle_boost_, 0.0, 0.45 * step_length_);

    step_length_ = std::clamp(step_length_, min_step_length_, max_step_length_);
    if (step_period_ < min_step_period_) {
      step_period_ = min_step_period_;
    }
    crouch_deg_ = std::clamp(crouch_deg_, 5.0, 40.0);
    swing_lift_ = std::clamp(swing_lift_, 0.005, 0.08);

    // Feasibility: the worst-case chain is the touchdown pose - the
    // planting swing ankle sits (step_length + sagittal_margin) forward
    // of its hip while the support chain already spans d1 (partial
    // shift). Check BOTH explicitly: the previous guard only checked
    // the support chain and the touchdown chain silently exceeded the
    // 0.97*leg_len limit -> clamped targets -> contact fight (AT-11).
    const double z0 = leg_len_ * std::cos(crouch_deg_ * kDeg);
    const double d1 =
      std::sqrt(support_shift_ * support_shift_ + z0 * z0);
    const double reach_td = std::sqrt(
      (step_length_ + sagittal_margin_) *
      (step_length_ + sagittal_margin_) + d1 * d1);
    if (reach_td > 0.97 * leg_len_ || z0 <= thigh_ * 0.5) {
      RCLCPP_FATAL(
        get_logger(),
        "gait_generator: unreachable gait geometry (step_length=%.3f, "
        "crouch_deg=%.1f: touchdown chain %.3f m > %.3f m). Refusing "
        "to start.",
        step_length_, crouch_deg_, reach_td, 0.97 * leg_len_);
      state_ = GaitState::ESTOP;  // no trajectory will ever be produced
    }

    action_name_ = declare_parameter<std::string>(
      "action_name", "joint_trajectory_controller/follow_joint_trajectory");

    joints_ = {
      "left_hip_roll", "left_hip_pitch", "left_knee",
      "left_ankle_pitch", "left_ankle_roll",
      "right_hip_roll", "right_hip_pitch", "right_knee",
      "right_ankle_pitch", "right_ankle_roll"};

    // URDF limits (leg.xacro properties) for the guard clamps.
    limits_ = {
      {deg(-25.0), deg(25.0)},    // left_hip_roll
      {deg(-90.0), deg(90.0)},    // left_hip_pitch
      {0.0, deg(100.0)},          // left_knee
      {deg(-50.0), deg(50.0)},    // left_ankle_pitch
      {deg(-25.0), deg(25.0)},    // left_ankle_roll
      {deg(-25.0), deg(25.0)},    // right_hip_roll
      {deg(-90.0), deg(90.0)},    // right_hip_pitch
      {0.0, deg(100.0)},          // right_knee
      {deg(-50.0), deg(50.0)},    // right_ankle_pitch
      {deg(-25.0), deg(25.0)}};   // right_ankle_roll

    set_parameter(rclcpp::Parameter("use_sim_time", true));

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&GaitGenerator::joint_state_cb, this, std::placeholders::_1));

    // Pelvis IMU (M8): world-referenced orientation, used to keep the
    // soles world-flat (the stance feet must grip with their full
    // 16x9 cm patch, not rock on an edge).
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10,
      std::bind(&GaitGenerator::imu_cb, this, std::placeholders::_1));

    cmd_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/gait_command", 10,
      std::bind(&GaitGenerator::gait_command_cb, this, std::placeholders::_1));

    support_pub_ = create_publisher<std_msgs::msg::String>("/gait_support", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/gait_status", 10);

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, action_name_);

    timer_ = create_wall_timer(100ms, std::bind(&GaitGenerator::spin_once, this));

    RCLCPP_INFO(
      get_logger(),
      "gait_generator: step_length=%.3f m, step_period=%.2f s, crouch=%.1f deg, "
      "z0=%.3f m (starts with the %s leg). std_msgs/Bool on /gait_command: "
      "true=walk, false=stop.", step_length_, step_period_, crouch_deg_, z0,
      start_first_left_ ? "left" : "right");
  }

  ~GaitGenerator() override
  {
    if (action_client_ && goal_handle_) {
      action_client_->async_cancel_goal(goal_handle_);
    }
  }

private:
  struct JointLimit
  {
    double lower;
    double upper;
  };

  // One interpolated segment of a step.
  struct Segment
  {
    enum class Tag
    {
      APPROACH,
      ROLL,
      SHIFT,
      LIFT,
      SWING,
      PLANT,
      SETTLE
    };
    Tag tag;
    std::vector<double> pose;
    double duration;
    std::string support;  // STANCE foot "", "left", "right" published on
                          // /gait_support while the segment executes
  };

  static double deg(double d) {return d * kDeg;}

  // ---- state input ----

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
      have_state_ = true;
    }
  }

  // World-referenced roll/pitch of the pelvis (ZYX Euler from the IMU
  // quaternion), low-pass filtered (tau ~ 50 ms at 100 Hz) to reject
  // sample spikes while still tracking the weight-transfer wobble.
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & q = msg->orientation;
    const double roll = std::atan2(
      2.0 * (q.w * q.x + q.y * q.z),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    const double pitch = std::asin(
      std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
    const double az = msg->linear_acceleration.z;
    const double wy = msg->angular_velocity.y;
    if (!have_imu_) {
      imu_roll_ = roll;
      imu_pitch_ = pitch;
      imu_az_ = az;
      imu_gyro_y_ = wy;
      have_imu_ = true;
      return;
    }
    constexpr double kAlpha = 0.2;
    imu_roll_ += kAlpha * (roll - imu_roll_);
    imu_pitch_ += kAlpha * (pitch - imu_pitch_);
    imu_az_ += kAlpha * (az - imu_az_);
    imu_gyro_y_ += kAlpha * (wy - imu_gyro_y_);
  }

  // True only when the body is provably AT REST and upright: vertical
  // acceleration near gravity, gyro near zero, attitude level. Used to
  // gate walk (re-)starts - the AT-11 startup-thrash fix. The spawn-drop
  // transient aborts the first approach and every ~2 s re-arm attempt
  // (az-only gate passed mid-flap), manufacturing 318 aborts with ZERO
  // steps built (run BI). A moving body must never start a crouch.
  bool body_at_rest() const
  {
    return have_imu_ && std::abs(imu_az_ - 9.81) < 1.0 &&
           std::abs(imu_gyro_y_) < 0.10 &&
           std::abs(imu_roll_) < 0.44 && std::abs(imu_pitch_) < 0.44;
  }

  void gait_command_cb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      if (state_ == GaitState::ESTOP) {
        if (estop_recovery_active_) {
          RCLCPP_WARN(
            get_logger(), "gait_command: recovery glide in progress - ignored");
          return;
        }
        // AT-11 run AM: a mid-walk tracking abort freezes the pose while
        // the body is still upright - a NEW operator walk request may
        // re-arm from there (fresh APPROACH crouch from the frozen pose).
        // A genuinely fallen body (|roll| or |pitch| > ~25 deg) stays
        // latched: safety gate, not auto-resume - the request itself is
        // the operator action.
        if (body_at_rest()) {
          RCLCPP_WARN(
            get_logger(),
            "gait_command: walk re-armed from ESTOP (body upright)");
          state_ = GaitState::WALKING;
          ever_walked_ = true;
          build_approach();
        } else {
          RCLCPP_WARN(
            get_logger(), "gait_command: true ignored - fallen body in ESTOP");
        }
        return;
      }
      if (state_ == GaitState::IDLE) {
        if (!body_at_rest()) {
          RCLCPP_WARN(
            get_logger(),
            "gait_command: WALK ignored - body not at rest (az=%.2f, "
            "gyro_y=%.3f)", imu_az_, imu_gyro_y_);
          return;
        }
        RCLCPP_INFO(get_logger(), "gait_command: WALK requested");
        state_ = GaitState::WALKING;
        ever_walked_ = true;  // closes the startup auto-recovery window
        build_approach();
      } else {
        RCLCPP_INFO(get_logger(), "gait_command: already walking");
      }
    } else {
      if (state_ == GaitState::WALKING) {
        state_ = GaitState::STOPPING;  // handled at the next segment boundary
        seg_retries_ = 0;  // fresh retry budget for the landing squat
        RCLCPP_INFO(
          get_logger(),
          "gait_command: STOP requested - landing swing foot, then idle");
      } else if (state_ == GaitState::IDLE) {
        RCLCPP_INFO(get_logger(), "gait_command: stop ignored (already idle)");
      } else {
        RCLCPP_WARN(get_logger(), "gait_command: stop while %s", state_name(state_));
      }
    }
  }

  // ---- IK and pose building ----

  // Flat-sole 2-link IK in the sagittal plane (leg.xacro convention:
  // pitch about +Y; knee flexion positive; ankle pitch keeps the sole
  // parallel to the pelvis). hip_x: ankle x position relative to the
  // pelvis (+forward); drop: vertical pelvis-to-ankle distance.
  void leg_ik(double hip_x, double drop, size_t idx, std::vector<double> & q) const
  {
    const double l1 = thigh_;
    const double l2 = shin_;
    const double d2 = hip_x * hip_x + drop * drop;
    const double d = std::sqrt(std::max(d2, 1e-12));
    double cos_k = (d2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
    cos_k = std::clamp(cos_k, -1.0, 1.0);
    const double knee = std::acos(cos_k);
    const double gamma = std::atan2(hip_x, std::max(drop, 1e-6));
    double sin_a = l2 * std::sin(knee) / std::max(d, 1e-6);
    sin_a = std::clamp(sin_a, -1.0, 1.0);
    // Knee flexion pushes the shin toward -x of the thigh line, so the
    // thigh must point PAST the hip->ankle line (+x side):
    // a1 = -(phi + alpha), i.e. a NEGATIVE hip pitch for a forward
    // (+x) ankle target (pitch about +Y: +angle swings the leg to -x).
    // The opposite sign commanded the swing foot backward - the robot
    // walked -x; found by cross-checking the M6 odometry (leg_fk).
    const double hip = -(gamma + std::asin(sin_a));
    q[idx] = hip;
    q[idx + 1] = knee;
    q[idx + 2] = -(hip + knee);
  }

  // Build a full 10-joint target. sup_left selects which leg is the
  // support leg; roll is the lateral weight shift applied to BOTH hips
  // (ankle rolls compensate so both soles stay flat).
  std::vector<double> build_pose(
    bool sup_left,
    double sup_hip_x, double sup_drop,
    double sw_hip_x, double sw_drop,
    double roll) const
  {
    std::vector<double> q(joints_.size(), 0.0);
    const size_t sup = sup_left ? 0u : 5u;   // index of *_hip_roll
    const size_t sw = sup_left ? 5u : 0u;
    q[sup] = roll;
    q[sup + 4] = -roll;
    q[sw] = roll;
    q[sw + 4] = -roll;
    leg_ik(sup_hip_x, sup_drop, sup + 1, q);
    leg_ik(sw_hip_x, sw_drop, sw + 1, q);
    return q;
  }

  void clamp_targets(std::vector<double> & q)
  {
    for (size_t i = 0; i < q.size(); ++i) {
      const double raw = q[i];
      q[i] = std::clamp(q[i], limits_[i].lower, limits_[i].upper);
      if (std::abs(q[i] - raw) > 1e-9) {
        clamp_events_++;
        if (clamp_events_ <= 20 || clamp_events_ % 100 == 0) {
          RCLCPP_WARN(
            get_logger(), "joint-limit guard: %s %.3f -> %.3f (event #%llu)",
            joints_[i].c_str(), raw, q[i],
            static_cast<unsigned long long>(clamp_events_));
        }
      }
    }
  }

  double z_stand() const {return leg_len_ * std::cos(crouch_deg_ * kDeg);}

  // Single-support planar drop: the pelvis rests support_shift_ INBOARD
  // of the support ankle (partial lateral shift - the CoM is inside the
  // foot polygon, no full roll-over needed), so the pitch chain spans
  // d1 = sqrt(shift^2 + z0^2) at roll = asin(shift/d1). At the shallow
  // crouches near the AT-05-proven 15 deg stand the full roll-over span
  // sqrt((hip_y-shift)^2 + z0^2) is kinematically UNREACHABLE (it
  // forced crouch >= 25 deg, and even there the touchdown chain
  // sqrt((L+m)^2 + d1^2) exceeded the 0.97*leg limit -> clamped,
  // inconsistent targets, contact fight). The partial shift keeps the
  // chain within ~z0 at any feasible crouch.
  double drop_ss() const
  {
    return std::sqrt(
      support_shift_ * support_shift_ + z_stand() * z_stand());
  }
  // Roll shift sign: the pelvis moves TOWARD the support foot (and
  // support_overshoot_ PAST it, inboard), so both feet move to -Y
  // relative to the pelvis when the support is the left leg (rotation
  // about +X moves a hanging foot toward +Y). Both legs get the same
  // roll, so both feet keep their world-y: the planted trailing foot
  // stays put (it just unloads as the pelvis crosses over it).
  double roll_ss(double d1) const
  {
    return (swing_left_ ? 1.0 : -1.0) *
           std::asin(std::clamp(support_shift_ / d1, -1.0, 1.0));
  }
  // Roll shift sign: the pelvis moves TOWARD the support foot (and
  // support_overshoot_ PAST it, inboard), so both feet move to -Y
  // relative to the pelvis when the support is the left leg (rotation
  // about +X moves a hanging foot toward +Y). Both legs get the same
  // roll, so both feet keep their world-y: the planted trailing foot
  // stays put (it just unloads as the pelvis crosses over it).
  // Append a lateral weight shift as sub-waypoints on the exact
  // planted-feet curve. With both soles flat and planted, the roll
  // angle and chain span are NOT independent: loop closure gives
  // span(t) = z0 / cos(roll(t)) (lateral rel = z0*tan(roll)).
  // Commanding the final span d1 while the roll still interpolates
  // (the previous implementation) is inconsistent by up to 9 mm -
  // the joints fight the contacts from the first update. Sub-waypoints
  // every quarter of the shift keep every commanded pose exactly
  // consistent; the JTC's joint-space interpolation between them is
  // then a close approximation of the true curve.
  void append_roll_transition(
    bool sup_left, double roll_from, double roll_to,
    double hx_sup_from, double hx_sup_to,
    double hx_sw_from, double hx_sw_to,
    double total_duration, const std::string & support)
  {
    constexpr int kSub = 4;
    const double z0 = z_stand();
    for (int i = 1; i <= kSub; ++i) {
      const double f = static_cast<double>(i) / kSub;
      const double roll = roll_from + f * (roll_to - roll_from);
      const double drop =
        z0 / std::cos(std::clamp(std::abs(roll), 1e-6, 1.2));
      Segment seg;
      seg.tag = roll_to != 0.0 ? Segment::Tag::SHIFT : Segment::Tag::SETTLE;
      seg.pose = build_pose(
        sup_left,
        hx_sup_from + f * (hx_sup_to - hx_sup_from), drop,
        hx_sw_from + f * (hx_sw_to - hx_sw_from), drop,
        roll);
      clamp_targets(seg.pose);
      seg.duration = total_duration / kSub;
      seg.support = support;
      segments_.push_back(seg);
    }
  }

  // Segment queue for one step. Step 0 starts from feet-together (a half
  // step); later steps use the steady 5-segment cycle.
  void build_step()
  {
    segments_.clear();
    seg_i_ = 0;
    trimmed_ = false;
    seg_retries_ = 0;  // fresh retry budget per step

    // Iterative-learning sag update, ONCE per step, here at the step
    // boundary: latest_positions_ is the settled quasi-static DOUBLE-
    // SUPPORT equilibrium (the JTC holds the SETTLE target), the one
    // state where the load distribution is known and repeatable.
    // Updating per-segment instead poisoned the gait: sag learned in
    // the symmetric APPROACH transferred into the asymmetric SHIFT,
    // the progressively unloading leg got a command extended past
    // plan with no load to resist it, and the body was launched
    // (measured as +45/-45 g flip-flops at the first shift). The
    // learned offset then rides unchanged through the whole step.
    if (have_state_ && !last_nominal_.empty() && ilc_gain_ != 0.0 &&
      ilc_updates_ < ilc_freeze_after_)
    {
      ++ilc_updates_;
      for (size_t i = 0; i < ilc_.size(); ++i) {
        const double err = last_nominal_[i] - latest_positions_[i];
        ilc_[i] = std::clamp(
          ilc_[i] + ilc_gain_ * err, -kIlcMax, kIlcMax);
      }
      RCLCPP_INFO(
        get_logger(),
        "gait_generator: ILC update #%d (freeze after %d)",
        ilc_updates_, ilc_freeze_after_);
    }

    const double z0 = z_stand();
    const double L = step_length_;
    const bool sup_left = !swing_left_;
    const double d1 = drop_ss();

    // Swing-phase durations SCALE WITH THE STEP PERIOD (AT-11 run H:
    // LIFT/SWING/PLANT used fixed 0.1/0.3/0.1 s - proportioned for the
    // original ~2 s step. At step_period=10 s the swing ankle had to
    // sweep 0.46 rad in 0.3 s: the JTC aborted on Position Error
    // 0.3005 > 0.300 (right_ankle_pitch, joint 8) and the gait froze
    // at the same step every run.)
    const double t_lift = frac_[1] * step_period_;
    const double t_swing = frac_[2] * step_period_;
    const double t_plant = frac_[3] * step_period_;
    const double ff = hip_sag_ff_;  // DS hip feedforward (ramped)
    const double roll = roll_ss(d1);
    const double lift = swing_lift_;
    const double m = sagittal_margin_;  // pelvis rests m BEHIND support ankle
    const double b = settle_boost_;     // sag-compensating over-advance

    if (step_index_ == 0) {
      // Feet together: the pelvis creeps BACK sagittal_margin_ (both
      // feet planted), the swing foot lifts straight up, then swings
      // one full step ahead. Roll comes up while both feet are down.
      // Every single-support leg (support AND the foot being planted)
      // spans drop = d1 in the rolled plane: the pelvis stays at the
      // FULL height z0 (d1*cos(roll) = z0) and both planted feet rest
      // exactly on the ground - no pelvis sag, no hover, no settle
      // push (the M8 bounce was exactly that: drop=z0 legs rolled
      // 16 deg sagged the pelvis 13.6 mm, then SETTLE commanded it
      // back up = one pogo stroke per step).
      segments_.push_back(
        {Segment::Tag::ROLL, build_pose(sup_left, 0.0, z0, 0.0, z0, 0.0),
          1e-6, stance_name(sup_left)});
      // ROLL: lateral shift onto the support foot along the
      // planted-feet curve (the pelvis also creeps back to the
      // sagittal margin while both feet are down).
      append_roll_transition(
        sup_left, 0.0, roll, 0.0, m, 0.0, m,
        frac_[0] * step_period_, stance_name(sup_left));
      segments_.push_back(
        {Segment::Tag::LIFT,
         build_pose(sup_left, m, d1, m, z0 - lift, roll),
          t_lift, stance_name(sup_left)});
      segments_.push_back(
        {Segment::Tag::SWING,
         build_pose(sup_left, m, d1, L + m, z0 - 0.5 * lift, roll),
          t_swing, stance_name(sup_left)});
      segments_.push_back(
        {Segment::Tag::PLANT,
         build_pose(sup_left, m, d1, L + m, d1, roll),
          t_plant, stance_name(sup_left)});
      // SETTLE: roll back down on the planted-feet curve while the
      // pelvis advances to settle_boost_ PAST the new feet midpoint
      // (the sag recoil then lands it ON the midpoint - AJ measured
      // the pelvis falling 0.03-0.05 m behind per commanded cycle).
      append_roll_transition(
        sup_left, roll, 0.0, m, -(0.5 * L + b), L + m, 0.5 * L - b,
        frac_[4] * step_period_, "");
    } else {
      // Steady cycle: support is the FRONT foot (rel pelvis +L/2 at the
      // end of the previous settle), swing is the BACK foot (-L/2).
      // SHIFT advances the pelvis L/2 - m + b (from b past the feet
      // midpoint - where the previous SETTLE left it - to
      // sagittal_margin_ behind the front ankle) along the
      // planted-feet roll curve: the trailing foot is never dragged
      // (its rel target -(L-m) keeps it world-fixed), and the CoM stays
      // inside the polygon through the whole segment.
      append_roll_transition(
        sup_left, 0.0, roll, 0.5 * L - b, m, -(0.5 * L + b), -(L - m),
        frac_[0] * step_period_, stance_name(sup_left));
      segments_.push_back(
        {Segment::Tag::LIFT,
         build_pose(sup_left, m, d1, -(L - m), z0 - lift, roll),
          t_lift, stance_name(sup_left)});
      segments_.push_back(
        {Segment::Tag::SWING,
         build_pose(sup_left, m, d1, L + m, z0 - 0.5 * lift, roll),
          t_swing, stance_name(sup_left)});
      segments_.push_back(
        {Segment::Tag::PLANT,
         build_pose(sup_left, m, d1, L + m, d1, roll),
          t_plant, stance_name(sup_left)});
      append_roll_transition(
        sup_left, roll, 0.0, m, -(0.5 * L + b), L + m, 0.5 * L - b,
        frac_[4] * step_period_, "");
    }

    // Hip sag feedforward: CONSTANT on every segment (both hips).
    // Evidence: run Q (constant 0.13 via send-time add) was the calmest
    // progress run; run X (binary DS gate at send time) stepped the
    // hip target by the full FF at the PLANT->SETTLE boundary -
    // instant 0.13 rad jump, body yank, |a| 77 spikes. Baked into the
    // poses here so every segment including the APPROACH carries the
    // identical offset - no boundary steps anywhere.
    for (auto & s : segments_) {
      s.pose[1] += ff;
      s.pose[6] += ff;
      clamp_targets(s.pose);
    }
    // NOTE: support_ is NOT pre-set here. spin_once() publishes
    // /gait_support when it sees seg.support change, so the odometry is
    // notified of every stance transition at the segment boundary.
    RCLCPP_INFO(
      get_logger(), "gait_generator: step %d (%u segments) - %s swings",
      step_index_ + 1, static_cast<unsigned>(segments_.size()),
      swing_name(sup_left).c_str());
  }

  void build_approach()
  {
    segments_.clear();
    seg_i_ = 0;
    trimmed_ = false;
    seg_retries_ = 0;
    step_index_ = 0;
    steps_since_start_ = 0;
    swing_left_ = start_first_left_;
    // Symmetric crouch, pelvis centered between the (planted) feet.
    const double z0 = z_stand();
    std::fill(ilc_.begin(), ilc_.end(), 0.0);  // fresh learning window
    ilc_updates_ = 0;
    Segment seg;
    seg.tag = Segment::Tag::APPROACH;
    seg.pose = build_pose(true, 0.0, z0, 0.0, z0, 0.0);
    seg.pose[1] += hip_sag_ff_;   // both hips carry the FF from the
    seg.pose[6] += hip_sag_ff_;   // start (APPROACH is pure DS)
    clamp_targets(seg.pose);
    seg.duration = approach_time_;
    seg.support = "";
    segments_.push_back(seg);
    in_approach_ = true;    support_.clear();
  }

  std::string swing_name(bool sup_left) const
  {
    return sup_left ? "right" : "left";
  }

  // The stance (planted) leg - the leg that is NOT swinging.
  std::string stance_name(bool sup_left) const
  {
    return sup_left ? "left" : "right";
  }

  // STOP: drop segments so the swing foot lands and the pose settles.
  void trim_for_stop()
  {
    trimmed_ = true;
    size_t plant_idx = segments_.size();
    for (size_t i = 0; i < segments_.size(); ++i) {
      if (segments_[i].tag == Segment::Tag::PLANT) {
        plant_idx = i;
        break;
      }
    }
    if (plant_idx == segments_.size()) {
      segments_.clear();  // nothing left worth executing
      return;
    }
    // Keep PLANT + all SETTLE sub-waypoints (the settle is a sequence
    // of roll-curve waypoints - skipping any of them jumps the pose).
    const std::vector<Segment> kept(
      segments_.begin() + static_cast<long>(plant_idx), segments_.end());
    segments_ = kept;
    seg_i_ = 0;
  }

  // ---- action client ----

  bool try_send(const Segment & seg)
  {
    if (!action_client_ || !action_client_->action_server_is_ready() ||
      awaiting_result_)
    {
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = joints_;

    // Point 0 = current measured pose with zero velocity so a frozen
    // goal commands no motion. (No effort field: the position-mode JTC
    // rejects trajectories that carry effort values.)
    trajectory_msgs::msg::JointTrajectoryPoint start;
    start.positions = have_state_ ? latest_positions_ : seg.pose;
    start.velocities.assign(joints_.size(), 0.0);
    start.time_from_start = rclcpp::Duration::from_seconds(0.0);
    goal.trajectory.points.push_back(start);

    trajectory_msgs::msg::JointTrajectoryPoint target;
    target.positions = seg.pose;

    // Iterative-learning sag compensation (AT-11). The soft servo that
    // calmed the 25 Hz contact pump sags under load: the stance knee
    // (~8-15 N*m) settles ~0.13 rad short and the stance ankle leans
    // the pelvis ~4 deg - the pelvis never reaches the planned pose,
    // so the step geometry never accumulates into forward progress.
    // Feed 0.5x of the PREVIOUS segment's end error into this target
    // (capped): one-step-per-segment integrator on the command, driven
    // by the actual load, no plant model. Reset on ESTOP/approach.
    if (have_state_) {
      for (size_t i = 0; i < target.positions.size(); ++i) {
        target.positions[i] += ilc_[i];
      }
    }
    // NOTE: hip_sag_ff is baked into the POSES (progressive ramp in
    // append_roll_transition + full in build_approach), not added at
    // send time - see the ramp comment there.

    // Swing-sole levelling (AT-11 treadmill fix). The open-loop IK
    // keeps the soles PARALLEL TO THE PELVIS: foot_attitude =
    // pelvis_attitude + sum(pitch/roll chain angles), and the IK makes
    // each sum zero - so any pelvis lean rocks the feet onto their
    // edges and the soles stop gripping. Compensation is sign-correct
    // ONLY for the free foot: commanding sum = -pelvis_attitude lands
    // the swing sole world-flat. On the STANCE foot the same command
    // deletes the servo's attitude-restoring error (the lean then runs
    // away - measured as a +45/-45 deg flip-flop) - so only the ankle
    // of the leg NAMED IN seg.support's opposite swings is compensated.
    // DS segments (support "") get no levelling: with both feet pinned
    // the contacts, not the servos, hold the attitude.
    double dp = 0.0, dr = 0.0;
    if (have_imu_ && sole_level_gain_ != 0.0) {
      int sw_pitch = -1, sw_roll = -1;
      if (seg.support == "left") {        // left = stance, right swings
        sw_pitch = static_cast<int>(kRAnklePitch);
        sw_roll = static_cast<int>(kRAnkleRoll);
      } else if (seg.support == "right") {
        sw_pitch = static_cast<int>(kLAnklePitch);
        sw_roll = static_cast<int>(kLAnkleRoll);
      }
      if (sw_pitch >= 0) {
        dp = std::clamp(
          -sole_level_gain_ * imu_pitch_, -kMaxLevel, kMaxLevel);
        dr = std::clamp(
          -sole_level_gain_ * imu_roll_, -kMaxLevel, kMaxLevel);
        target.positions[static_cast<size_t>(sw_pitch)] += dp;
        target.positions[static_cast<size_t>(sw_roll)] += dr;
      }
    }

    target.velocities.assign(joints_.size(), 0.0);
    target.time_from_start =
      rclcpp::Duration::from_seconds(std::max(0.05, seg.duration));
    goal.trajectory.points.push_back(target);

    auto options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    options.goal_response_callback =
      [this](const GoalHandleFollowJointTrajectory::SharedPtr & gh) {
        if (!gh) {
          RCLCPP_ERROR(get_logger(), "gait_generator: goal REJECTED");
          awaiting_result_ = false;
          estop("goal rejected by controller");
        }
      };
    options.result_callback =
      [this](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
        awaiting_result_ = false;
        goal_handle_ = nullptr;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            if (estop_recovery_active_) {
              // Recovery glide reached the symmetric stand: IDLE again.
              // The rest-gated commander can now start a clean session
              // (the stand IS the stable rest pose, unlike the old
              // frozen feet-apart pose that never rested - run BK).
              estop_recovery_active_ = false;
              state_ = GaitState::IDLE;
              RCLCPP_INFO(
                get_logger(),
                "gait_generator: recovered to symmetric stand - IDLE");
            }
            break;
          case rclcpp_action::ResultCode::ABORTED:
            if (estop_recovery_active_) {
              // The recovery glide needs retries too (sag-aware resend,
              // same semantics as a walk segment): from a feet-apart
              // frozen pose the crouch-to-stand tracking needs 1-2
              // resends; without this the FIRST tracking abort hard-
              // froze every recovery (runs BL-BS).
              if (recovery_retries_ < 3) {
                ++recovery_retries_;
                awaiting_result_ = false;
                RCLCPP_WARN(
                  get_logger(),
                  "gait_generator: recovery glide tracking transient "
                  "(retry %d/3) - re-sending", recovery_retries_);
                try_send(recovery_seg_);
              } else {
                hard_freeze("recovery glide aborted");
              }
            } else if (!transient_retry_ok("tracking transient")) {
              estop("trajectory aborted (tracking constraints)");
            }
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "gait_generator: trajectory canceled");
            break;
          default:
            estop("unknown action result");
            break;
        }
      };

    awaiting_result_ = true;
    seg_deadline_ = now() + rclcpp::Duration::from_seconds(seg.duration + 5.0);
    last_nominal_ = seg.pose;

    // AT-11 diagnostics: full commanded pose per segment (compare
    // against /joint_states and the gz pose stream to separate
    // commanded motion from physical foot sliding).
    std::string qstr;
    for (double qv : target.positions) {
      qstr += " " + std::to_string(qv);
    }
    RCLCPP_INFO(
      get_logger(), "gait_generator: send seg tag=%d dur=%.3f lvl=(%.3f, %.3f) q=%s",
      static_cast<int>(seg.tag), seg.duration, dp, dr, qstr.c_str());
    action_client_->async_send_goal(goal, options);
    return true;
  }

  void cancel_active_goal(const char * why)
  {
    if (goal_handle_) {
      action_client_->async_cancel_goal(goal_handle_);
      RCLCPP_WARN(get_logger(), "gait_generator: goal canceled (%s)", why);
      goal_handle_ = nullptr;
    }
  }

  // Final freeze at the current measured pose (the classic ESTOP of
  // section 12) - now only the LAST resort, when even the recovery
  // glide to stand fails (genuinely falling body).
  void hard_freeze(const char * why)
  {
    estop_recovery_active_ = false;
    RCLCPP_ERROR(
      get_logger(), "gait_generator: recovery FAILED (%s) - pose frozen", why);
    if (have_state_) {
      Segment hold;
      hold.tag = Segment::Tag::SETTLE;
      hold.pose = latest_positions_;
      hold.duration = 0.05;
      hold.support = "";
      try_send(hold);
    }
  }

  // Emergency stop (section 12), now with the AT-11 run-BY discovery:
  // if the body is objectively UPRIGHT (|roll|,|pitch| < 25 deg), the
  // verified-safe landing is the AT-10 graceful-stop machinery - land
  // the swing foot, settle on both feet, IDLE (sessions can re-chain).
  // The old behavior froze mid-gait (feet apart = never at rest, run
  // BK: no second session could ever start); the interim recovery-
  // glide-to-stand failed every time (BL-BX: dragging the feet under
  // a loaded, off-center body needs strength the plant does not have).
  // A genuinely tipped body still hard-freezes (safety, section 12).
  void estop(const std::string & reason)
  {
    if (state_ == GaitState::ESTOP) {
      return;
    }
    cancel_active_goal(reason.c_str());
    awaiting_result_ = false;
    segments_.clear();
    std::fill(ilc_.begin(), ilc_.end(), 0.0);  // learned sag was for the
    support_.clear();                          // aborted maneuver only
    publish_support();
    if (have_state_ && have_imu_ &&
        std::abs(imu_roll_) < 0.44 && std::abs(imu_pitch_) < 0.44) {
      RCLCPP_ERROR(
        get_logger(), "gait_generator: ESTOP - %s (upright: graceful land)",
        reason.c_str());
      state_ = GaitState::STOPPING;
      seg_retries_ = 0;  // fresh retry budget for the landing squat
      trimmed_ = false;  // spin_once trims: PLANT + SETTLE from the queue
      // Rebuild the remainder if the abort ate the queue: the trim
      // needs a PLANT segment to exist. Fresh step from the current
      // pose (same as an ESTOP re-arm, but ending in a SETTLE, not a
      // crouch hold).
      if (segments_.empty()) {
        build_step();
        trimmed_ = false;
      }
      publish_status();
      return;
    }
    state_ = GaitState::ESTOP;
    publish_status();
    RCLCPP_ERROR(get_logger(), "gait_generator: ESTOP - %s (tipped: pose frozen)", reason.c_str());
    if (have_state_) {
      Segment hold;
      hold.tag = Segment::Tag::SETTLE;
      hold.pose = latest_positions_;
      hold.duration = 0.05;
      hold.support = "";
      try_send(hold);
    }
    // AT-11 run AI discovery: the model DROP at sim startup can trip the
    // JTC state-tolerance abort before any gait code runs (spawn settle
    // bounce). The random half of the A/B runs died this way - "calm"
    // runs and violent runs differed by spawn luck, not config.
    // Bounded recovery for the PRE-WALK window only: while no walk has
    // ever been requested, resume the stand hold. A walking robot NEVER
    // auto-resumes (AT-10 safety intact: ESTOP during walk stays latched).
    if (!ever_walked_) {
      ++startup_estops_;
      if (startup_estops_ <= 10) {
        RCLCPP_WARN(
          get_logger(),
          "gait_generator: startup ESTOP %d/10 (pre-walk) - resuming stand hold",
          startup_estops_);
        state_ = GaitState::IDLE;
      }
    }
  }

  // ---- main loop ----

  void publish_support()
  {
    std_msgs::msg::String m;
    m.data = support_;
    support_pub_->publish(m);
  }

  void publish_status()
  {
    std_msgs::msg::String m;
    m.data = state_name(state_);
    status_pub_->publish(m);
  }

  // AT-11 run AP: with the soles friction-pinned, hip/knee gravity sag
  // (0.13-0.3 rad) can exceed the JTC goal tolerance - the segment never
  // reports SUCCEEDED and the watchdog fires. Retry the segment (up to
  // 8x) while the IMU says the body is still upright; a real fall
  // (> ~25 deg) or exhausted budget latches ESTOP as before.
  // 2026-09-21 (view3): STOPPING now retries too - the graceful-stop
  // landing is the DEEPEST squat of the whole cycle (max sag transient
  // risk), and the WALKING-only gate let the watchdog kill its own
  // landing 8 s in and freeze the robot mid-motion. Same budget, same
  // IMU-upright gate: a genuinely tipping body still latches ESTOP.
  bool transient_retry_ok(const char * why)
  {
    // Budget 8 (was 3): a loaded step's 5 segments collectively meet
    // 3-5 sag transients (goal tol 0.30 vs measured 0.13-0.3 rad sag);
    // the old 3 exhausted mid-step at the 4th transient and killed the
    // session while the robot was objectively upright (|a| = 9.8 at
    // death, run BS) - the "3-step session limit" was arithmetic, not
    // balance. Each retry is still IMU-gated upright; a real fall
    // latches ESTOP exactly as before.
    if ((state_ == GaitState::WALKING || state_ == GaitState::STOPPING) &&
        seg_retries_ < 8 &&
        have_imu_ && std::abs(imu_roll_) < 0.44 &&
        std::abs(imu_pitch_) < 0.44) {
      ++seg_retries_;
      --seg_i_;
      awaiting_result_ = false;
      RCLCPP_WARN(
        get_logger(),
        "gait_generator: %s while upright (retry %d/8) - re-sending segment",
        why, seg_retries_);
      return true;
    }
    return false;
  }

  void spin_once()
  {
    publish_status();

    if (state_ != GaitState::WALKING && state_ != GaitState::STOPPING) {
      return;
    }
    if (!have_state_) {
      return;
    }

    if (awaiting_result_) {
      if (now() > seg_deadline_) {
        // Same IMU-gated retry as a tracking abort: a soft-servo segment
        // can outlive its deadline while objectively upright; a tipped
        // body (or exhausted retries) latches ESTOP.
        if (estop_recovery_active_) {
          if (!transient_retry_ok("recovery deadline")) {
            hard_freeze("recovery watchdog timeout");
          }
        } else if (!transient_retry_ok("segment deadline exceeded")) {
          estop("segment watchdog timeout");
        }
      }
      return;
    }
    if (state_ == GaitState::ESTOP) {
      return;
    }

    // Stop handling at segment boundaries.
    if (state_ == GaitState::STOPPING && !trimmed_) {
      trim_for_stop();
    }

    if (seg_i_ >= segments_.size()) {
      step_finished();
      return;
    }

    const Segment & seg = segments_[seg_i_];
    if (seg.support != support_) {
      support_ = seg.support;
      publish_support();
    }
    if (try_send(seg)) {
      seg_i_++;
    }
  }

  void step_finished()
  {
    if (in_approach_) {
      in_approach_ = false;
      build_step();
      return;
    }

    steps_since_start_++;
    swing_left_ = !swing_left_;
    step_index_++;

    if (state_ == GaitState::STOPPING) {
      RCLCPP_INFO(
        get_logger(), "gait_generator: stop complete after %d step(s)",
        steps_since_start_);
      state_ = GaitState::IDLE;
      support_.clear();
      publish_support();
      publish_status();
      return;
    }

    if (walk_distance_limit_ > 0.0) {
      const double advanced =
        (steps_since_start_ > 0 ? steps_since_start_ - 0.5 : 0.0) * step_length_;
      if (advanced >= walk_distance_limit_) {
        RCLCPP_INFO(
          get_logger(),
          "gait_generator: walk_distance_limit (%.2f m) reached - stopping",
          walk_distance_limit_);
        state_ = GaitState::STOPPING;
        return;
      }
    }

    build_step();
  }

  // ---- members ----
  // joints_ order: hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll
  // per leg, left block first.
  static constexpr size_t kLAnklePitch = 3;
  static constexpr size_t kRAnklePitch = 8;
  static constexpr size_t kLAnkleRoll = 4;
  static constexpr size_t kRAnkleRoll = 9;
  static constexpr double kMaxLevel = 0.15;  // rad, ~8.6 deg safety clamp
  static constexpr double kIlcMax = 0.12;    // rad anti-windup cap

  double thigh_, shin_, leg_len_;
  double step_length_, step_period_, approach_time_, swing_lift_;
  double hip_y_, crouch_deg_;
  double support_overshoot_;
  double support_shift_;
  double sagittal_margin_;
  double settle_boost_;
  std::vector<double> frac_;
  double max_step_length_, min_step_length_, min_step_period_;
  bool start_first_left_;
  double walk_distance_limit_;
  std::string action_name_;

  std::vector<std::string> joints_;
  std::vector<JointLimit> limits_;

  GaitState state_{GaitState::IDLE};
  bool estop_recovery_active_{false};
  Segment recovery_seg_;
  int recovery_retries_{0};
  std::vector<Segment> segments_;
  size_t seg_i_{0};
  bool trimmed_{false};
  bool in_approach_{false};
  int step_index_{0};
  int steps_since_start_{0};
  bool swing_left_{true};
  std::string support_;
  bool ever_walked_{false};  // startup-ESTOP auto-recovery window gate
  int startup_estops_{0};
  int seg_retries_{0};       // IMU-gated per-step tracking-transient retries

  std::vector<double> latest_positions_;
  std::vector<double> ilc_ = std::vector<double>(10, 0.0);
  std::vector<double> last_nominal_;
  bool have_state_{false};
  bool have_imu_{false};
  double imu_roll_{0.0};
  double imu_pitch_{0.0};
  double imu_az_{9.81};
  double imu_gyro_y_{0.0};
  double sole_level_gain_;
  double ilc_gain_;
  int ilc_freeze_after_;
  int ilc_updates_{0};
  double hip_sag_ff_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  bool awaiting_result_{false};
  rclcpp::Time seg_deadline_{0, 0, RCL_ROS_TIME};
  uint64_t clamp_events_{0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr support_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  GoalHandleFollowJointTrajectory::SharedPtr goal_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GaitGenerator>());
  rclcpp::shutdown();
  return 0;
}
