cmake_minimum_required(VERSION 3.10)
project(so_arm_teleop)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(trajectory_msgs REQUIRED)

install(PROGRAMS
  scripts/keyboard_joint_selector.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
