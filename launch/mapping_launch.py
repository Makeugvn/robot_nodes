import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('robot_nodes')

    xacro_file = os.path.join(pkg, 'urdf', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    urdf_tmp = '/tmp/robot.urdf'
    with open(urdf_tmp, 'w') as f:
        f.write(robot_desc)

    world_file  = os.path.join(pkg, 'worlds', 'corridor.sdf')
    slam_params = os.path.join(pkg, 'config', 'slam_params.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_file    = os.path.join(pkg, 'maps', 'map.yaml')

    return LaunchDescription([

        # ── Robot state publisher ──────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': True}],
            output='screen'
        ),

        # ── Static TF: base_link → laser ──────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.05', '0', '0.06',
                       '0', '0', '0',
                       'base_link', 'laser'],
            name='base_to_laser_tf'
        ),

        # ── Gazebo ────────────────────────────────────────
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_file],
            output='screen'
        ),

        # ── Spawn robot ───────────────────────────────────
        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_ign_gazebo', 'create',
                     '-name', 'robot', '-file', urdf_tmp,
                     '-x', '-1.5', '-y', '0.0', '-z', '0.05'],
                output='screen'
            ),
        ]),

        # ── Bridge Gazebo → ROS 2 ─────────────────────────
        # /tf_static hanya SATU baris — tidak duplikat
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            ],
            output='screen'
        ),

        # ── SLAM toolbox ──────────────────────────────────
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params, {'use_sim_time': True}],
            output='screen'
        ),

        # ── Nav2 (delay 5s agar SLAM + odom siap) ─────────
        TimerAction(period=5.0, actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[nav2_params,
                             {'yaml_filename': map_file,
                              'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                parameters=[nav2_params, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                parameters=[nav2_params, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                parameters=[nav2_params, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                parameters=[nav2_params, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                parameters=[nav2_params, {'use_sim_time': True}],
                output='screen'
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': [
                        'map_server',
                        'amcl',
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                    ]
                }],
                output='screen'
            ),
        ]),

        # ── State machine (delay 8s agar Nav2 siap) ───────
        TimerAction(period=8.0, actions=[
            Node(
                package='robot_nodes',
                executable='state_machine_node.py',
                name='state_machine_node',
                output='screen'
            ),
        ]),

        # ── RViz2 ─────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])