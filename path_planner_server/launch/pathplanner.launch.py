import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    package_name = "path_planner_server"

    nav2_yaml = os.path.join(get_package_share_directory("localization_server"), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory('cartographer_slam'), 'maps', 'real_map.yaml')

    # RVIZ configuration file
    rviz_file = "path_planner.rviz"
    rviz_config_dir = os.path.join(get_package_share_directory("path_planner_server"), "rviz", rviz_file)


    controller_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'planner_server.yaml')
    behavior_server_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'behavior_server.yaml')



    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
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
                        {'use_sim_time': False}]
        ),        
        

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings={
                ('/cmd_vel', '/turtlebot_5/cmd_vel')
            },
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]
        ),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[behavior_server_yaml],
            output='screen',
            remappings={
                ('/cmd_vel', '/turtlebot_5/cmd_vel')
            },
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': [
                                        'map_server',
                                        'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'behavior_server',
                                        'bt_navigator']}]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            parameters=[{
                "use_sim_time": False
            }],
            arguments=[
                "-d", rviz_config_dir
            ]
        ),

    ])
