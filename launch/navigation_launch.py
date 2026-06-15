#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg         = get_package_share_directory('robot_nodes')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_file    = os.path.join(pkg, 'maps', 'map.yaml')

    return LaunchDescription([

        # ── Static TF ──────────────────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'base_footprint', 'base_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_tf',
            arguments=['0.05', '0', '0.10', '0', '0', '0',
                       'base_link', 'laser'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_tf',
            arguments=['0', '0', '0.05', '0', '0', '0',
                       'base_link', 'imu_link'],
        ),

        # ── Odometry + Lidar ───────────────────────────────
        Node(
            package='robot_nodes',
            executable='odometry_node.py',
            name='odometry_node',
            output='screen',
        ),
        Node(
            package='robot_nodes',
            executable='rplidar_wifi.py',
            name='lidar_serial_node',
            parameters=[{
                'frame_id': 'laser',
                'range_min': 0.10,
                'range_max': 12.0,
            }],
            output='screen',
        ),

        # ── cmd_vel bridge: /cmd_vel → ESP32 UDP port 5007 ─
        Node(
            package='robot_nodes',
            executable='cmd_vel_bridge.py',
            name='cmd_vel_bridge',
            parameters=[{
                'esp32_ip':   '192.168.100.94',
                'esp32_port': 5007,
            }],
            output='screen',
        ),

        # ── Map server ─────────────────────────────────────
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                nav2_params,
                {'yaml_filename': map_file},
            ],
            output='screen',
        ),

        # ── AMCL — broadcast TF map → odom ─────────────────
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[nav2_params],
            output='screen',
        ),

        # ── Nav2 stack ─────────────────────────────────────
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params],
            output='screen',
            remappings=[('cmd_vel', '/cmd_vel')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params],
            output='screen',
        ),

        # ── Lifecycle manager ──────────────────────────────
        # Urutan penting: map_server → amcl → sisanya
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
        ),

        # ── RViz2 ──────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])