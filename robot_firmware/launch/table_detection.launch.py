import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    package_name = "robot_firmware"


    return LaunchDescription([
        
        Node(
            package=package_name, 
            executable='table_detection', 
            name='table_detection',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        Node(
            package=package_name, 
            executable='main_node', 
            name='main_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),


    ]) 
