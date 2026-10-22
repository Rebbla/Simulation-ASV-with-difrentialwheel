cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/camera_vision_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
