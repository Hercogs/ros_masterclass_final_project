import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    package_name = "robot_firmware"


    # RVIZ configuration file
    rviz_file = "rvis.rviz"
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file)
    rviz_config_dir = "/home/user/ros2_ws/src/rvis.rviz"

    return LaunchDescription([
        
        Node(
            package=package_name, 
            executable='table_detection', 
            name='table_detection',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            parameters=[{
                "use_sim_time": True,
            }],
            arguments=[
                "-d ", rviz_config_dir
            ]
        )

    ]) 
