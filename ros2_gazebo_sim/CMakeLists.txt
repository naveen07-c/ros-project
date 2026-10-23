cmake_minimum_required(VERSION 3.8)
project(biped_gazebo_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/gait_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
