import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, SetParameter

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    #params_file = LaunchConfiguration('params_file')
    package_dir      = get_package_share_directory("emcl2")
    emcl_params_file = os.path.join(package_dir, "config", 'emcl2_params.yaml')
    #use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file         = os.path.join(package_dir, "map", 'map.yaml')
    print('\nMAP',map_file)

    rviz = os.path.join(package_dir, "rviz" , "nav2_default_view.rviz")
    print('\nRVIZ2 PATH',rviz)


    lifecycle_nodes = ['map_server']
    

    return LaunchDescription([

        # map server node
        Node(
            package    = 'nav2_map_server',
            executable = 'map_server',
            name       = 'map_server',
            output     = 'screen',
            parameters = [{'yaml_filename': map_file, 
                           'use_sim_time' : False
                         }],
        ),

        # emcl node
        Node(
            package    = 'emcl2',
            name       = 'emcl2',
            executable = 'emcl2_node',
            output     = 'screen',
            parameters = [emcl_params_file],
        ),

        # static TF conversion (base_link -> laser_frame)
        Node(
            package    = 'tf2_ros',
            executable = 'static_transform_publisher',
            output     = 'screen',
            arguments  = ['0.0', '0.0', '0.2', '0.0', '0.0', '0.0', 'base_link', 'laser_frame'],
        ),

        # rviz node
        Node(
            package    = 'rviz2',
            executable = 'rviz2',
            name       = 'rviz2',
            arguments  = ['-d', rviz],
            parameters = [{'use_sim_time': False}],
            output     = 'screen'
        ),

        # life cycle node: manages the life cycle states of nodes
        Node(
            package    = 'nav2_lifecycle_manager',
            executable = 'lifecycle_manager',
            name       = 'lifecycle_manager_localization',
            output     = 'screen',
            parameters = [{'autostart': True},
                          {'node_names': lifecycle_nodes}]
        ),
    ])
