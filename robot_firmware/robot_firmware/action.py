#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from robot_firmware_interfaces.action import AproachTable  # import the action message
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor


class SimpleActionServer(Node):

    def __init__(self):
        super().__init__('main_node')
        self._action_server = ActionServer(
            self,
            AproachTable,
            'aproach_table',
            self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Perform the desired action here
        # ...

        # Check if the action has been canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return

        # Check if the action has been preempted
        # if goal_handle.is_preempt_requested:
        #     goal_handle.abort()
        #     self.get_logger().info('Goal preempted')
        #     return

        # Set the result of the action
        result = AproachTable.Result()
        result.result = 1
        
        goal_handle.succeed()
        return result

        self.get_logger().info('Goal completed')

def main(args=None):
    rclpy.init(args=args)
    simple_action_server = SimpleActionServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(simple_action_server)

    executor.spin()
    executor.shutdown()

    rclpy.shutdown()

if __name__ == '__main__':
    main()


