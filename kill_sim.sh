#!/bin/bash
pkill -9 -f "gz" || true
pkill -9 -f "gazebo" || true
pkill -9 -f "rviz2" || true
pkill -9 -f "ros_gz" || true
pkill -9 -f "parameter_bridge" || true
pkill -9 -f "waypoint_patrol" || true
pkill -9 -f "camera_vision" || true
pkill -9 -f "robot_state_publisher" || true
pkill -9 -f "static_transform_publisher" || true
pkill -9 -f "create" || true
echo "Semua proses simulasi telah dimatikan bersih."
