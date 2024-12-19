from launch import LaunchDescription
import os
from ament_index_python import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():

    # Declare the arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    map = LaunchConfiguration('map')

    # Define RViz config path
    rviz_config_path = os.path.join(
        get_package_share_directory('locker98_tools_bringup'),
        'rviz', 'nav2_config.rviz'
    )

    # Include the bringup launch file
    bringup_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'bringup_launch.py'
    )

    # Include the bringup launch file and pass arguments
    include_bringup = IncludeLaunchDescription(
        bringup_launch_file,
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map
        }.items()
    )

    # RViz node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Return LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
        DeclareLaunchArgument('map', default_value='', description='Map input'),
        include_bringup,
        rviz2_node
    ])
