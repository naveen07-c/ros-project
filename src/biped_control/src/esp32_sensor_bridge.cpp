// Copyright 2026 Biped project (M12 Tier S)
//
// esp32_sensor_bridge — reads the ESP32 telemetry stream (UART1, 115200)
// and republishes it on the exact topics the simulation nodes consume, so
// gait_odometry, contact_markers and gait_generator run byte-identical on
// hardware (HARDWARE_SELECTION.md §7.3).
//
//   /imu                    sensor_msgs/Imu   (~50 Hz, frame_id = base_link;
//                             orientation synthesized from complementary-
//                             filtered accel tilt + gyro, yaw = 0 — matches
//                             the gz IMU sensor's gravity-referenced output)
//   /left_foot_contact      ros_gz_interfaces/Contacts (published ONLY while
//   /right_foot_contact       the sole FSRs report load — silence is the
//                             swing signal, identical to the gz sensors)
//
// Stream format (one CSV line per sample, '\n'-terminated):
//   T,<ms>,<ax_mm>,<ay_mm>,<az_mm>,<gx_md>,<gy_md>,<gz_md>,<fl>,<fr>
//   accel in mm/s^2, gyro in mdeg/s, FSR 12-bit ADC counts 0..4095.

#include <cerrno>
#include <chrono>
#include <cmath>>
#include <cstring>
#include <string>
#include <vector>

#include "fcntl.h"
#include "termios.h"
#include "unistd.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
#include "ros_gz_interfaces/msg/contact.hpp"

namespace biped_control
{

class Esp32SensorBridge : public rclcpp::Node
{
public:
  Esp32SensorBridge()
  : Node("esp32_sensor_bridge")
  {
    device_ = declare_parameter<std::string>("serial_device", "/dev/ttyUSB1");
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    // FSR calibration: below adc_threshold = no contact; above it, force is
    // scaled linearly to adc_full_scale counts == fs_full_newton.
    adc_threshold_ = declare_parameter<double>("adc_threshold", 250.0);
    adc_full_scale_ = declare_parameter<double>("adc_full_scale", 3500.0);
    fs_full_newton_ = declare_parameter<double>("fs_full_newton", 40.0);
    // Complementary-filter blend for the synthesized orientation (same
    // structure as the sim's gz IMU plugin: accel tilt low-passed, gyro
    // integrated between corrections).
    alpha_ = declare_parameter<double>("imu_alpha", 0.98);
    period_ = declare_parameter<double>("publish_period", 0.02);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    left_pub_ = create_publisher<ros_gz_interfaces::msg::Contacts>(
      "/left_foot_contact", 10);
    right_pub_ = create_publisher<ros_gz_interfaces::msg::Contacts>(
      "/right_foot_contact", 10);

    if (!open_serial()) {
      RCLCPP_FATAL(
        get_logger(), "cannot open ESP32 telemetry port %s — retrying every 2 s",
        device_.c_str());
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_),
      std::bind(&Esp32SensorBridge::poll, this));
  }

private:
  bool open_serial()
  {
    fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
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
    RCLCPP_INFO(get_logger(), "ESP32 telemetry on %s", device_.c_str());
    return true;
  }

  void poll()
  {
    if (fd_ < 0 && !open_serial()) {
      return;
    }
    char buf[512];
    ssize_t n;
    while ((n = ::read(fd_, buf, sizeof(buf))) > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        if (buf[i] == '\n') {
          line_.push_back('\0');
          parse_and_publish(line_.data());
          line_.clear();
        } else if (buf[i] != '\r') {
          if (line_.size() < 256) {
            line_.push_back(buf[i]);
          } else {
            line_.clear();  // overrun: drop the malformed line, resync
          }
        }
      }
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      ::close(fd_);
      fd_ = -1;
    }
    // Publish the cached IMU sample at the fixed ROS rate (the stream is
    // ~50 Hz; republishing the latest keeps /imu continuous for the
    // consumers' freshness gates).
    publish_imu();
    publish_contacts();
  }

  bool token(const char * s, int idx, double & out) const
  {
    int k = 0;
    while (*s) {
      if (k == idx) {
        out = atof(s);
        return true;
      }
      while (*s && *s != ',') {
        ++s;
      }
      if (*s == ',') {
        ++s;
      }
      ++k;
    }
    return k == idx ? (out = 0.0, true) : false;
  }

  void parse_and_publish(char * s)
  {
    double v[11];
    for (int i = 0; i < 11; ++i) {
      if (!token(s, i, v[i])) {
        return;  // short/malformed line: drop silently
      }
    }
    // v: T, ms, ax, ay, az, gx, gy, gz, fl, fr  (indices 0..9)
    ax_ = v[2] / 1000.0;  // mm/s^2 -> m/s^2
    ay_ = v[3] / 1000.0;
    az_ = v[4] / 1000.0;
    gx_ = v[5] * M_PI / 180.0 / 1000.0;  // mdeg/s -> rad/s
    gy_ = v[6] * M_PI / 180.0 / 1000.0;
    gz_ = v[7] * M_PI / 180.0 / 1000.0;
    f_left_ = fsr_to_newton(v[8]);
    f_right_ = fsr_to_newton(v[9]);
    have_data_ = true;
  }

  double fsr_to_newton(double adc) const
  {
    if (adc < adc_threshold_) {
      return 0.0;
    }
    return std::min(
      (adc - adc_threshold_) / (adc_full_scale_ - adc_threshold_) *
      fs_full_newton_,
      fs_full_newton_);
  }

  void publish_imu()
  {
    if (!have_data_) {
      return;
    }
    // Complementary attitude: gyro integrates, accel tilt corrects. While
    // the body accelerates the accel tilt is unreliable, hence the strong
    // gyro weight (0.98) — the same structure as the sim's IMU behavior.
    const double accel_pitch = std::atan2(-ax_, std::hypot(ay_, az_));
    const double accel_roll = std::atan2(ay_, az_);
    const double dt = period_;
    pitch_ = alpha_ * (pitch_ + gy_ * dt) + (1.0 - alpha_) * accel_pitch;
    roll_ = alpha_ * (roll_ + gx_ * dt) + (1.0 - alpha_) * accel_roll;

    // ZYX quaternion with yaw = 0 (yaw is unobservable without a magnetometer;
    // every consumer here uses roll/pitch only).
    const double cr = std::cos(roll_ * 0.5), sr = std::sin(roll_ * 0.5);
    const double cp = std::cos(pitch_ * 0.5), sp = std::sin(pitch_ * 0.5);

    sensor_msgs::msg::Imu m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.orientation.w = cp * cr;
    m.orientation.x = cp * sr;
    m.orientation.y = sp * cr;
    m.orientation.z = -sp * sr;
    m.angular_velocity.y = gy_;
    m.angular_velocity.x = gx_;
    m.angular_velocity.z = gz_;
    m.linear_acceleration.x = ax_;
    m.linear_acceleration.y = ay_;
    m.linear_acceleration.z = az_;
    imu_pub_->publish(m);
  }

  void publish_contacts()
  {
    // Contact semantics identical to gz: publish ONLY while loaded; silence
    // = swing. Two FSR channels per sole sum into one contact entry whose
    // body_1_wrench force magnitude is what gait_odometry thresholds on.
    publish_foot(left_pub_, f_left_, "left_foot_collision");
    publish_foot(right_pub_, f_right_, "right_foot_collision");
  }

  void publish_foot(
    const rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr & pub,
    double f_newton, const char * collision_name)
  {
    if (!have_data_ || f_newton <= 0.0) {
      return;
    }
    ros_gz_interfaces::msg::Contacts m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    ros_gz_interfaces::msg::Contact c;
    c.collision1.name = std::string("biped::") + collision_name;
    c.collision2.name = "ground_plane::link::collision";
    auto & jw = c.wrenches.emplace_back();
    jw.body_1_name.data = c.collision1.name;
    jw.body_1_wrench.force.z = f_newton;
    m.contacts.push_back(c);
    pub->publish(m);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr left_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr right_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string device_, frame_id_;
  double adc_threshold_, adc_full_scale_, fs_full_newton_, alpha_, period_;
  double ax_{0}, ay_{0}, az_{0}, gx_{0}, gy_{0}, gz_{0};
  double f_left_{0}, f_right_{0};
  double roll_{0}, pitch_{0};
  bool have_data_{false};
  std::string line_;
  int fd_{-1};
};

}  // namespace biped_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<biped_control::Esp32SensorBridge>());
  rclcpp::shutdown();
  return 0;
}
