// Copyright 2026 Biped project (M12 Tier S)
//
// pwm_servo_hw — ros2_control hardware interface for the ₹15,000 student
// build (HARDWARE_SELECTION.md §7). Command path: this node (PC, ROS 2
// Jazzy) --USB serial 115200--> ESP32 firmware (firmware/esp32_firmware.ino)
// --I2C--> PCA9685 --> 10 × DS3218 PWM servos.
//
// Interface contract:
//   command: POSITION on all 10 joints (µs pulses, per-joint trim map).
//   state:   position = last commanded (PWM servos have NO position
//            readback — standard practice), velocity = finite difference,
//            effort = 0 (no torque feedback; documented accepted loss).
//
// Serial protocol (one line per frame, '\n'-terminated):
//   PC -> ESP32 : "P,<us0>,<us1>,...,<us9>\n"   (10 pulse widths, 500..2500)
//   ESP32 -> PC : telemetry handled by esp32_sensor_bridge on a second
//                 USB-UART (UART1), so this port is write-only in practice;
//                 we still drain any echoed bytes to keep the line clean.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>

#include "fcntl.h"
#include "termios.h"
#include "unistd.h"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pwm_servo_hw
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PwmServoHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PwmServoHardware)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }
    logger_ = rclcpp::get_logger("PwmServoHardware");

    device_ = info_.hardware_parameters.count("serial_device")
      ? info_.hardware_parameters.at("serial_device") : "/dev/ttyUSB0";
    // Per-joint pulse map. For each joint, in the URDF's ros2_control block:
    //   <parameter name="channel">0..9</parameter>
    //   <parameter name="center_us">1500</parameter>
    //   <parameter name="us_per_rad">600</parameter>
    //   <parameter name="dir">1 | -1</parameter>
    n_joints_ = info_.joints.size();
    if (n_joints_ != 10) {
      RCLCPP_FATAL(
        logger_, "expected 10 joints (Tier S biped), got %zu", n_joints_);
      return CallbackReturn::ERROR;
    }
    channel_.resize(n_joints_);
    center_us_.resize(n_joints_);
    us_per_rad_.resize(n_joints_);
    dir_.resize(n_joints_);
    for (size_t j = 0; j < n_joints_; ++j) {
      const auto & p = info_.joints[j].parameters;
      channel_[j] = p.count("channel") ? std::stoi(p.at("channel")) : (int)j;
      center_us_[j] = p.count("center_us") ? std::stoi(p.at("center_us")) : 1500;
      us_per_rad_[j] = p.count("us_per_rad") ? std::stod(p.at("us_per_rad")) : 600.0;
      dir_[j] = p.count("dir") ? std::stoi(p.at("dir")) : 1;
      if (info_.joints[j].command_interfaces.size() != 1 ||
          info_.joints[j].command_interfaces[0].name != "position")
      {
        RCLCPP_FATAL(
          logger_, "joint %s must claim exactly one POSITION command interface",
          info_.joints[j].name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    cmd_.assign(n_joints_, 0.0);
    pos_state_.assign(n_joints_, 0.0);
    vel_state_.assign(n_joints_, 0.0);
    eff_state_.assign(n_joints_, 0.0);  // no torque sensing — documented loss
    last_us_.assign(n_joints_, -1);  // force first full frame
    RCLCPP_INFO(logger_, "pwm_servo_hw ready on %s (%zu joints)", device_.c_str(), n_joints_);
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> out;
    for (size_t j = 0; j < n_joints_; ++j) {
      out.emplace_back(info_.joints[j].name, "position", &pos_state_[j]);
      out.emplace_back(info_.joints[j].name, "velocity", &vel_state_[j]);
      out.emplace_back(info_.joints[j].name, "effort", &eff_state_[j]);
    }
    return out;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> out;
    for (size_t j = 0; j < n_joints_; ++j) {
      out.emplace_back(info_.joints[j].name, "position", &cmd_[j]);
    }
    return out;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    if (!open_serial()) {
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!open_serial()) {
      return CallbackReturn::ERROR;
    }
    // Stand pose: hold whatever the neutral centers are until the first
    // controller command arrives (the spawn hold_joints analog).
    for (size_t j = 0; j < n_joints_; ++j) {
      pos_state_[j] = 0.0;
    }
    RCLCPP_INFO(logger_, "activated; sending neutral pulse frame");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (fd_ >= 0) {
      // Neutral centers = servos release to a known pose; the physical
      // E-stop (servo-rail toggle, §12) remains the real kill switch.
      std::string line = "P";
      for (size_t j = 0; j < n_joints_; ++j) {
        line += (j ? "," : "") + std::to_string(center_us_[j]);
      }
      line += "\n";
      (void)::write(fd_, line.c_str(), line.size());
      ::close(fd_);
      fd_ = -1;
    }
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration & period) override
  {
    // No encoder feedback: report commanded position, finite-diff velocity.
    for (size_t j = 0; j < n_joints_; ++j) {
      const double prev = pos_state_[j];
      pos_state_[j] = cmd_[j];
      vel_state_[j] =
        period.seconds() > 0.0 ? (pos_state_[j] - prev) / period.seconds() : 0.0;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (fd_ < 0) {
      return hardware_interface::return_type::ERROR;
    }
    // Build the pulse frame; skip joints whose µs did not change (keeps the
    // 50 Hz line well under bandwidth).
    std::string line;
    bool any = false;
    for (size_t j = 0; j < n_joints_; ++j) {
      long us = std::lround(
        center_us_[j] + dir_[j] * us_per_rad_[j] * cmd_[j]);
      us = std::max(500L, std::min(2500L, us));
      if (us != last_us_[j]) {
        line += "S" + std::to_string(channel_[j]) + "," +
                std::to_string(us) + "\n";
        last_us_[j] = us;
        any = true;
      }
    }
    if (any) {
      const ssize_t rc = ::write(fd_, line.c_str(), line.size());
      if (rc < 0) {
        RCLCPP_ERROR_THROTTLE(
          logger_, clock_, 2000, "serial write failed; reconfiguring");
        ::close(fd_);
        fd_ = -1;
        return hardware_interface::return_type::ERROR;
      }
    }
    return hardware_interface::return_type::OK;
  }

private:
  bool open_serial()
  {
    if (fd_ >= 0) {
      return true;
    }
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(logger_, "cannot open %s", device_.c_str());
      return false;
    }
    termios tty{};
    if (::tcgetattr(fd_, &tty) != 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      ::close(fd_);
      fd_ = -1;
      return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
  }

  rclcpp::Logger logger_{rclcpp::get_logger("PwmServoHardware")};
  rclcpp::Clock clock_;
  std::string device_;
  size_t n_joints_{0};
  std::vector<int> channel_, center_us_, dir_;
  std::vector<double> us_per_rad_;
  std::vector<double> cmd_, pos_state_, vel_state_;
  std::vector<double> eff_state_;
  std::vector<long> last_us_;
  int fd_{-1};
};

}  // namespace pwm_servo_hw

PLUGINLIB_EXPORT_CLASS(
  pwm_servo_hw::PwmServoHardware, hardware_interface::SystemInterface)
