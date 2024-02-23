#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.duration import Duration

from rclpy.action import ActionClient

from robot_firmware_interfaces.action import AproachTable  # import the action message
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tf_transformations  import quaternion_from_euler

# ros2 run robot_firmware test_nav2 --ros-args -p use_sim_time:=true

# Home position
home_position = {
    # [x, y, theta - angle to rotate when robot is under table]
    "home_1": [7.25, -2.5, -1.57, 0.0],
}

# Shelf positions for picking
trash_table_position = {
    # [x, y, theta - angle to rotate when robot is under table]
    "table_1": [-0.25, 1.20, 0.0, -1.57],
}

# osition(4.85357, -0.901903, 0), Orientation(0, 0, 0.0523186, 0.99863) = Angle: 0.104685
# Position(0.375583, -0.514267, 0), Orientation(0, 0, -0.995858, 0.090919) =Angle: -2.9595
# osition(-1.44846, -0.505206, 0), Orientation(0, 0, 0.688796, 0.724956) = Angle: 1.51965


class MainNode(Node):

    def __init__(self):
        super().__init__('main_node')

        self.navigator = BasicNavigator()
        self.state = ""

        self.get_logger().info('Waiting for navigattion to start...')
        self.navigator.waitUntilNav2Active()

        # Create action client
        self.action_cli = ActionClient(self, AproachTable, 'aproach_table')

        # Create dummy timer
        self.timer = self.create_timer(1.0, self.timer_clb, callback_group=MutuallyExclusiveCallbackGroup())
        self.action_status = False

        # self.get_logger().info('Initializing robot pose...')
        # self.set_initial_position()

        self.get_logger().info('Main node started')


    def timer_clb(self):
        self.timer.cancel()
        self.action_status = False


        # set our demo's goal poses
        goal_poses = []
        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = 'map'
        goal_pose1.header.stamp = self.get_clock().now().to_msg()
        goal_pose1.pose.position.x = 4.85
        goal_pose1.pose.position.y = -0.9
        goal_pose1.pose.orientation.w = 0.0
        goal_pose1.pose.orientation.z = 1.0
        goal_poses.append(goal_pose1)

        # additional goals can be appended
        goal_pose2 = PoseStamped()
        goal_pose2.header.frame_id = 'map'
        goal_pose2.header.stamp = self.get_clock().now().to_msg()
        goal_pose2.pose.position.x = 0.37
        goal_pose2.pose.position.y = -0.5
        goal_pose2.pose.orientation.w = -1.0
        goal_pose2.pose.orientation.z = 0.0
        goal_poses.append(goal_pose2)

        goal_pose3 = PoseStamped()
        goal_pose3.header.frame_id = 'map'
        goal_pose3.header.stamp = self.get_clock().now().to_msg()
        goal_pose3.pose.position.x = -1.4
        goal_pose3.pose.position.y = -0.5
        goal_pose3.pose.orientation.w = 0.707
        goal_pose3.pose.orientation.z = 0.707
        goal_poses.append(goal_pose3)

        self.navigator.goThroughPoses(goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()


        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        

        return





        # Step 1 - go to known location where table might be
        table_loc = trash_table_position['table_1']
        result = self.go_to_pose(table_loc[0], table_loc[1], table_loc[2])
        if result != TaskResult.SUCCEEDED:
            print(f"Step 1 - go to known location where table might be: FAILED {result}")
            # TODO process failed task
            return
        print("Step 1 - go to known location where table might be: DONE")


        # Step 2 - trye to approach table
        (status, result) = self.aproach_table(dummy_aproach=True)
        if status != GoalStatus.STATUS_SUCCEEDED :
            print(f"action client failed: {status}")
            # TODO process failed task
            return
        
        # 0 - success, 1 - table not found, 2 - failed approach
        if result.result:
            print(f"Action cliented returned: {result.result}")
            # TODO process failed task -  go home
            return

        print(f"action server finished with result {result.result}")


         # Step 3 - Rotate robot
        result = self.rotate_robot(yaw=table_loc[3])
        if not result:
            print(f"Step 3 - Rotate robot: FAILED")
            # TODO process failed task
            return
        self.create_rate(1/(5*abs(table_loc[3]))).sleep()
        print(f"Step 3 - Rotate robot: SUCCESS")
        
        # Step 4 - Lift elevator
        # Step 4.1 - Change footprint


        # Step 5 - Move table to back room


        # Step 6 - Down elevator
        # Step 6.1 - Change footprint


        # Step 7 - Get out of table


        # Step 7 - Go to home position  
        home_loc = home_position['home_1']
        result = self.go_to_pose(home_loc[0], home_loc[1], home_loc[2])
        if result != TaskResult.SUCCEEDED:
            print(f"Step 7 - go to home position: FAILED {result}")
            # TODO process failed task
            return
        print("Step 7 - go to home positio: DONE")



        


    

    def set_initial_position(self):
        print("Set initial position")
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
    
    def go_to_pose(self, x, y, yaw):
        shelf_item_pose = PoseStamped()
        shelf_item_pose.header.frame_id = 'map'
        shelf_item_pose.header.stamp = self.get_clock().now().to_msg()
        shelf_item_pose.pose.position.x = x
        shelf_item_pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, yaw)

        shelf_item_pose.pose.orientation.x = q[0]
        shelf_item_pose.pose.orientation.y = q[1]
        shelf_item_pose.pose.orientation.z = q[2]
        shelf_item_pose.pose.orientation.w = q[3]
        print(f'Received request for item picking at {shelf_item_pose.pose}.')
        self.navigator.goToPose(shelf_item_pose)

        # Do something during our route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Simply print information for workers on the robot's ETA for the demonstation
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 20 == 0:
                # print(
                #     'Estimated time of arrival at '
                #     + str(shelf_item_pose.pose)
                #     + ' for worker: '
                #     + '{0:.0f}'.format(
                #         Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                #         / 1e9
                #     )
                #     + ' seconds.'
                # )
                print("Moving")

        result = self.navigator.getResult()
        return result
    
    def aproach_table(self, dummy_aproach):
        goal_msg = AproachTable.Goal()
        goal_msg.dummy_aproach = dummy_aproach

        available = self.action_cli.wait_for_server(timeout_sec=3)
        if not available:
            return(-1, None)
        
        res = self.action_cli.send_goal(goal_msg)
        status = res.status
        result = res.result
        return (status, result)

    

    def rotate_robot(self, yaw) -> bool:
        # return True if SUCCESS else False
        status = self.navigator.spin(spin_dist=yaw)
        return status


def main(args=None):
    rclpy.init(args=args)
    main_node = MainNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(main_node)

    executor.spin()
    executor.shutdown()

    rclpy.spin(main_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


