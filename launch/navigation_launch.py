import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    world_file = os.path.join(pkg, 'worlds', 'corridor.sdf')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': True}],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world_file],
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_ign_gazebo', 'create',
                        '-name', 'robot',
                        '-file', urdf_tmp,
                        '-x', '-1.5',
                        '-y', '0.0',
                        '-z', '0.05',
                    ],
                    output='screen'
                ),
            ]
        ),

        # Bridge — scan TIDAK di-bridge ke ROS saat navigasi
        # Hanya odom, cmd_vel, clock, dan tf
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=[
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/tf_static@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            ],
            output='screen'
        ),

        # Map server — load peta yang sudah disimpan
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': os.path.join(pkg, 'maps', 'corridor.yaml'),
            }],
            output='screen'
        ),

        # Lifecycle manager khusus map server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server'],
            }],
            output='screen'
        ),

        # Nav2 dengan params khusus navigasi (tanpa laser)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        get_package_share_directory('nav2_bringup'),
                        '/launch/navigation_launch.py'
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'params_file': os.path.join(
                            pkg, 'config', 'nav2_odom_only_params.yaml'),
                        'map_subscribe_transient_local': 'true',
                    }.items()
                ),
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
    ])