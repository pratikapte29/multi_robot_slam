import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ipb_ros2_sim')

    all_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'all.launch.py')
        ),
        launch_arguments={
            'sim_config': 'indoor_multi',
            'max_robot_count': '3',
        }.items()
    )

    robot_names = ['TT_robot0', 'TT_robot1', 'TT_robot2']
    nodes = []

    for robot_name in robot_names:
        nodes.append(Node(
            package='topic_tools',
            executable='relay',
            name=f'tf_relay_{robot_name}',
            arguments=[f'/{robot_name}/tf', '/tf'],
            output='screen',
        ))

        nodes.append(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': True,
                'scan_topic': f'/{robot_name}/lidar_2d',
                'odom_frame': f'{robot_name}/odom',
                'base_frame': f'{robot_name}/base_link',
                'map_frame': f'{robot_name}/map',
                'mode': 'mapping',
                'do_loop_closing': False,
                'resolution': 0.05,
                'max_laser_range': 20.0,
                'map_update_interval': 2.0,
                'transform_publish_period': 0.02,
                'transform_timeout': 0.2,
                'tf_buffer_duration': 30.0,
            }],
            output='screen',
        ))

    lifecycle_nodes = [
        '/TT_robot0/slam_toolbox',
        '/TT_robot1/slam_toolbox', 
        '/TT_robot2/slam_toolbox',
    ]

    for node_name in lifecycle_nodes:
        nodes.append(ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', node_name, 'configure'],
            output='screen',
            shell=True,
        ))
        nodes.append(ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', node_name, 'activate'],
            output='screen',
            shell=True,
        ))

    return LaunchDescription([all_launch] + nodes)