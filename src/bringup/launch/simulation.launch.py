#!/usr/bin/env python3
"""Launch file utama simulasi ROS 2 Jazzy Gazebo Harmonic + RViz2.

Dukungan Arena:
- ros2 launch bringup simulation.launch.py arena:=right (default: Map Kompak 8x8 m)
- ros2 launch bringup simulation.launch.py arena:=left  (Map Kompak 8x8 m Kiri)
- ros2 launch bringup simulation.launch.py arena:=full_right (Map Penuh 30x30 m Kanan)
- ros2 launch bringup simulation.launch.py arena:=full_left  (Map Penuh 30x30 m Kiri)
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    arena_val = context.launch_configurations.get('arena', 'right').lower()
    left_flag = context.launch_configurations.get('left', 'false').lower() == 'true'
    right_flag = context.launch_configurations.get('right', 'false').lower() == 'true'

    if left_flag:
        chosen_arena = 'left'
    elif right_flag:
        chosen_arena = 'right'
    else:
        chosen_arena = arena_val

    desc_pkg_share = get_package_share_directory('robot_description')
    nav_pkg_share = get_package_share_directory('navigasi')
    bringup_pkg_share = get_package_share_directory('bringup')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    urdf_file = os.path.join(desc_pkg_share, 'urdf', 'robot_patrol.urdf.xacro')
    rviz_config = os.path.join(bringup_pkg_share, 'rviz', 'simulation_debug.rviz')

    # Pemilihan Arena & Konfigurasi Waypoint
    if chosen_arena in ('full_left', 'fl'):
        world_filename = 'patrol_left_ground.sdf'
        config_filename = 'waypoints_left.yaml'
        spawn_x, spawn_y, spawn_z, spawn_yaw = '10.60', '-12.80', '0.05', '1.5708'
    elif chosen_arena in ('full_right', 'fr'):
        world_filename = 'patrol_right_ground.sdf'
        config_filename = 'waypoints_right.yaml'
        spawn_x, spawn_y, spawn_z, spawn_yaw = '-10.60', '-12.80', '0.05', '1.5708'
    elif chosen_arena in ('3x3_left', '3l'):
        world_filename = 'patrol_left_ground_3x3.sdf'
        config_filename = 'waypoints_left_3x3.yaml'
        spawn_x, spawn_y, spawn_z, spawn_yaw = '1.06', '-1.28', '0.03', '1.5708'
    elif chosen_arena in ('3x3_right', '3r'):
        world_filename = 'patrol_right_ground_3x3.sdf'
        config_filename = 'waypoints_right_3x3.yaml'
        spawn_x, spawn_y, spawn_z, spawn_yaw = '-1.06', '-1.28', '0.03', '1.5708'
    elif chosen_arena in ('left', 'l'):
        # Peta Kompak 8x8 m (Kiri)
        world_filename = 'patrol_left_ground_8x8.sdf'
        config_filename = 'waypoints_left_8x8.yaml'
        spawn_x, spawn_y, spawn_z, spawn_yaw = '2.83', '-3.41', '0.03', '1.5708'
    else:
        # Default: Peta Kompak 8x8 m (Kanan)
        world_filename = 'patrol_right_ground_8x8.sdf'
        config_filename = 'waypoints_right_8x8.yaml'
        spawn_x, spawn_y, spawn_z, spawn_yaw = '-2.83', '-3.41', '0.03', '1.5708'

    world_path = os.path.join(bringup_pkg_share, 'worlds', world_filename)
    config_path = os.path.join(bringup_pkg_share, 'config', config_filename)
    world_name = world_filename.replace('.sdf', '')

    # Proses xacro ke robot_description (gunakan package:// standar agar RViz & Gazebo memuat STL dengan benar)
    robot_description = subprocess.check_output(['xacro', urdf_file], text=True)

    # Resource path untuk Gazebo Sim & GUI agar mesh model STL selalu ditemukan
    share_root = os.path.dirname(desc_pkg_share)
    bringup_root = os.path.dirname(bringup_pkg_share)
    gz_resource_paths = [share_root, bringup_root]
    
    for var in ['GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH']:
        cur = os.environ.get(var, '')
        if cur:
            gz_resource_paths.append(cur)
        combined_path = os.pathsep.join(gz_resource_paths)
        os.environ[var] = combined_path

    env_gz_sim_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.environ['GZ_SIM_RESOURCE_PATH']
    )

    # 1. Gazebo Harmonic Node
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # 3. Spawn Robot di Dermaga (Cepat & Tahan race-condition dengan -world dan -string)
    spawn_robot = TimerAction(
        period=1.5,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='robot_spawner',
            arguments=[
                '-world', world_name,
                '-string', robot_description,
                '-name', 'robot_patrol',
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
            output='screen',
        )],
    )

    # 3b. Static TF Transform: world -> odom (Menyelaraskan koordinat Gazebo & RViz2 1:1)
    world_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_tf',
        output='screen',
        arguments=[
            '--x', spawn_x,
            '--y', spawn_y,
            '--z', '0.0',
            '--yaw', spawn_yaw,
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # 4. ROS-Gazebo Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # 5. Node Program Kamera (robot_description)
    camera_vision = TimerAction(
        period=5.0,
        actions=[Node(
            package='robot_description',
            executable='camera_vision_node.py',
            name='camera_vision_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    # 6. Node Navigasi Patrol (navigasi)
    navigasi_node = TimerAction(
        period=6.0,
        actions=[Node(
            package='navigasi',
            executable='waypoint_patrol_node.py',
            name='waypoint_patrol_node',
            output='screen',
            parameters=[
                config_path,
                {
                    'use_sim_time': True,
                    'autonomous': ParameterValue(
                        LaunchConfiguration('autonomous'), value_type=bool
                    ),
                },
            ],
        )],
    )

    # 7. RViz2
    rviz = TimerAction(
        period=5.5,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}],
        )],
    )

    return [
        env_gz_sim_path,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        world_to_odom_tf,
        bridge,
        camera_vision,
        navigasi_node,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'arena',
            default_value='right',
            description='Pilih arena Gazebo: "right" (8x8 m), "left" (8x8 m), "full_right" (30x30 m), "full_left" (30x30 m)',
            choices=['right', 'left', 'full_right', 'full_left', '3x3_right', '3x3_left']
        ),
        DeclareLaunchArgument(
            'left',
            default_value='false',
            description='Alias flag untuk memilih arena left: left:=true'
        ),
        DeclareLaunchArgument(
            'right',
            default_value='false',
            description='Alias flag untuk memilih arena right: right:=true'
        ),
        DeclareLaunchArgument(
            'autonomous',
            default_value='true',
            description='Aktifkan misi navigasi otonom (true/false)'
        ),
        OpaqueFunction(function=launch_setup),
    ])
