import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory("emcl2")
    emcl_params_file = os.path.join(package_dir, "config", 'emcl2_params.yaml')
    map_file = os.path.join(package_dir, "map", 'map.yaml')
    rviz = os.path.join(package_dir, "rviz", "nav2_default_view.rviz")

    lifecycle_nodes = ['map_server']

    rosbag_path = os.path.expanduser('~/ダウンロード/test_slam_1121_1732194107')

    return LaunchDescription([

        # Declare use_sim_time
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Map server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_file,
                'use_sim_time': True
            }],
        ),

        # emcl node
        Node(
            package='emcl2',
            name='emcl2',
            executable='emcl2_node',
            output='screen',
            parameters=[emcl_params_file, {'use_sim_time': True}],
        ),

        # Static TF conversion (base_link -> laser_frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser_frame'],
        ),

        # rviz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz],
            parameters=[{'use_sim_time': True}],  # Ensure sim time is used
            output='screen'
        ),

        # Life cycle manager node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': lifecycle_nodes}
            ],
        ),
    ])
