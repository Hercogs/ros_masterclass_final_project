#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from robot_firmware_interfaces.action import Empty  # import the action message
from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ros2 run robot_firmware main_node --ros-args -p use_sim_time:=true

# Shelf positions for picking
trash_table_position = {
    # [x, y, theta - angle to rotate when robot is under table]
    "table_1": [-0.25, 1.10, 0.0, -1.57],
}


class MainNode(Node):

    def __init__(self):
        super().__init__('main_node')

        self.navigator = BasicNavigator()
        self.state = ""

        self.get_logger().info('Waiting for navigattion to start...')
        self.navigator.waitUntilNav2Active()

        self.get_logger().info('Initializing robot pose...')
        self.set_initial_position()

        self.get_logger().info('Main node started')
        self.create_rate(1.0).sleep()  # Sleep for 1 sec
    

    def set_initial_position(self):
        # Set your demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.x = 0.0
        initial_pose.pose.orientation.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0

        self.navigator.setInitialPose(initial_pose)



def main(args=None):
    rclpy.init(args=args)
    simple_action_server = MainNode()
    rclpy.spin(simple_action_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


