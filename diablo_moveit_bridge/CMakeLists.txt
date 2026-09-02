cmake_minimum_required(VERSION 3.8)
project(diablo_moveit_bridge)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    diablo_moveit_bridge/ik_moveit_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ik_moveit_bridge
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
