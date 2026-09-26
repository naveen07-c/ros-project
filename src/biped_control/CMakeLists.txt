cmake_minimum_required(VERSION 3.8)
project(biped_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# M12 Tier S (HARDWARE_SELECTION.md §7): real-hardware layer.
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Standing-pose hold node: sends a static (or crouched) trajectory goal to
# the joint_trajectory_controller and keeps it active. (M5)
add_executable(stand_hold src/stand_hold.cpp)
target_compile_features(stand_hold PUBLIC cxx_std_17)
ament_target_dependencies(stand_hold
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  sensor_msgs
)

# Open-loop quasi-static gait generator (M6, AT-06/AT-10).
add_executable(gait_generator src/gait_generator.cpp)
target_compile_features(gait_generator PUBLIC cxx_std_17)
ament_target_dependencies(gait_generator
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  sensor_msgs
  std_msgs
)

# M9: RViz markers for the M8 sensor data (contact force arrows + IMU
# measured-up arrow).
add_executable(contact_markers src/contact_markers.cpp)
target_compile_features(contact_markers PUBLIC cxx_std_17)
ament_target_dependencies(contact_markers
  rclcpp
  sensor_msgs
  ros_gz_interfaces
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# Stance-foot kinematic odometry (M6, M8): /odom + odom->base_link TF.
# M8: stance source is the foot-contact sensors; pitch from the IMU.
add_executable(gait_odometry src/gait_odometry.cpp)
target_compile_features(gait_odometry PUBLIC cxx_std_17)
ament_target_dependencies(gait_odometry
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  ros_gz_interfaces
  tf2
  tf2_ros
)

# M12 Tier S: PWM servo hardware interface (ros2_control SystemInterface
# plugin; libpwm_servo_hw.so loaded via plugin/pwm_servo_hw.xml).
add_library(pwm_servo_hw SHARED src/pwm_servo_hw.cpp)
target_compile_features(pwm_servo_hw PUBLIC cxx_std_17)
ament_target_dependencies(pwm_servo_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# M12 Tier S: ESP32 sensor bridge — republishes the MPU6050/FSR telemetry
# stream as the exact /imu + /<side>_foot_contact topics the simulation
# nodes consume, so gait_generator/gait_odometry/contact_markers run
# byte-identical on hardware.
add_executable(esp32_sensor_bridge src/esp32_sensor_bridge.cpp)
target_compile_features(esp32_sensor_bridge PUBLIC cxx_std_17)
ament_target_dependencies(esp32_sensor_bridge
  rclcpp
  sensor_msgs
  ros_gz_interfaces
)

install(TARGETS stand_hold gait_generator gait_odometry contact_markers
  esp32_sensor_bridge
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pwm_servo_hw
  DESTINATION lib)

install(
  FILES plugin/pwm_servo_hw.xml
  DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY firmware
  DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(hardware_interface
  plugin/pwm_servo_hw.xml)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
