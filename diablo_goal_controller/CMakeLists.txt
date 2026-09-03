cmake_minimum_required(VERSION 3.8)
project(diablo_goal_controller)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  diablo_goal_controller/simple_goal_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
