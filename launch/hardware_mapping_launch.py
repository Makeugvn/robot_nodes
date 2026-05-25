#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('robot_nodes')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    return LaunchDescription([

        # ── Static TF: odom → base_footprint ──────────────
        # Robot statis, jadi odom tidak berubah
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'odom', 'base_footprint'],
            name='odom_to_base_tf'
        ),

        # ── Static TF: base_footprint → base_link ─────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_footprint', 'base_link'],
            name='base_footprint_to_base_tf'
        ),

        # ── Static TF: base_link → laser ──────────────────
        # Sesuaikan x,y,z dengan posisi sensor di robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.05', '0', '0.10', '0', '0', '0',
                       'base_link', 'laser'],
            name='base_to_laser_tf'
        ),

        # ── Lidar serial node ──────────────────────────────
        Node(
            package='robot_nodes',
            executable='lidar_serial_wifi.py',
            name='lidar_serial_node',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'frame_id': 'laser',
                'range_min': 0.10,
                'range_max': 12.0,
            }],
            output='screen'
        ),

        # ── SLAM toolbox ───────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_params,
                {'use_sim_time': False}   # hardware = real time
            ],
            output='screen'
        ),

        # ── RViz2 ──────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])