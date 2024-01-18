import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    package_name = "cartographer_slam"

    cartographer_config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    configuration_basename = 'cartographer.lua'

    # RVIZ configuration file
    rviz_file = "mapping.rviz"
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file)

    return LaunchDescription([
        
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            parameters=[{
                "use_sim_time": True,
            }],
            arguments=[
                "-d", rviz_config_dir
            ]
        ),

    ]) 
