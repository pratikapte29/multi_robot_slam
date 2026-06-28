import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import HasNodeParams
from launch.conditions import IfCondition

from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    pkg_share = get_package_share_directory('ipb_ros2_sim')

    nav2_params_file = os.path.join(
        get_package_share_directory('frontier_explorer'),
        'config', 'nav2_params.yaml'
    )

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
            remappings=[
                ('/map', f'/{robot_name}/map'),
                ('/map_metadata', f'/{robot_name}/map_metadata'),
            ],
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

        """
        below node was a mistake, basically all robots published on /map, 
        that caused them to overwrite each other one after other, and once they publish on /map
        it was relayed to each of their individual ones
        """

        # nodes.append(Node(
        #     package='topic_tools',
        #     executable='relay',
        #     name=f'map_relay_{robot_name}',
        #     namespace=robot_name,
        #     arguments=['/map', 'map'],
        #     output='screen',
        # ))

        nodes.append(Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            namespace=robot_name,
            parameters=[
                nav2_params_file, 
                {
                'controller_frequency': 20.0,
                'use_sim_time': True,
            }],
            output='screen',
        ))
        
        nodes.append(Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            namespace=robot_name,
            parameters=[
                nav2_params_file,
                {
                'expected_planner_frequency': 20.0,
                'use_sim_time': True,
            }],
            output='screen',
        ))
        
        nodes.append(Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            namespace=robot_name,
            parameters=[
                nav2_params_file,
                {
                'use_sim_time': True,
                'local_frame': f'{robot_name}/odom',
                'global_frame': f'{robot_name}/map',
                'robot_base_frame': f'{robot_name}/base_link',
            }],
            output='screen',
        ))
        
        nodes.append(Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            namespace=robot_name,
            parameters=[
                nav2_params_file,
                {
                'use_sim_time': True,
                'bt_xml_filename': '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml',
                'global_frame': f'{robot_name}/map',
                'robot_base_frame': f'{robot_name}/base_link',
                'odom_topic': f'/{robot_name}/odom',
            }],
            output='screen',
        ))

        nodes.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
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