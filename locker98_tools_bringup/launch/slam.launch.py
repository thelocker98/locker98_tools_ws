from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # Get package share directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    locker98_tools_bringup_dir = get_package_share_directory('locker98_tools_bringup')

    # Include the navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include the SLAM toolbox launch file
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(locker98_tools_bringup_dir, 'rviz', 'slam_config.rviz')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
        navigation_launch,
        slam_launch,
        rviz2_node
    ])
