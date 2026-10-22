cmake_minimum_required(VERSION 3.8)
project(navigasi)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/waypoint_patrol_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
