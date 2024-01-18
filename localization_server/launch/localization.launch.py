import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_name = "localization_server"
    
    nav2_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('cartographer_slam'), 'maps', 'sim_map.yaml')

    # RVIZ configuration file
    rviz_file = "localization.rviz"
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file)

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {"topic_name": "map"},
                        {"frame_id": "map"},
                        {'yaml_filename': map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml,
                        {'use_sim_time': True}]
        ),        
        
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            parameters=[{
                "use_sim_time": True
            }],
            arguments=[
                "-d", rviz_config_dir
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])
