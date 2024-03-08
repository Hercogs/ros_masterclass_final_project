#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from rclpy.action import ActionClient

from robot_firmware_interfaces.action import AproachTable  # import the action message
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tf_transformations  import quaternion_from_euler

from std_srvs.srv import SetBool

"""
colcon build --packages-select robot_firmware --symlink-install

ros2 run robot_firmware main_node
ros2 run robot_firmware table_detection

ros2 action send_goal /aproach_table robot_firmware_interfaces/action/AproachTable "dummy_aproach: false"

ros2 launch path_planner_server pathplanner.launch.py
"""


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


# -1.07, -0.5

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
        # self.timer = self.create_timer(1.0, self.timer_clb, callback_group=MutuallyExclusiveCallbackGroup())
        self.action_status = False
    
         # Create table subscriber
        self.table_sub = self.create_subscription(Bool, '/table', self.table_sub_clb, 3)
        self.is_table = False

        # self.get_logger().info('Initializing robot pose...')
        # self.set_initial_position()

        self.srv_clb_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(SetBool, '/move_trash_table', self.table_srv_clb, callback_group=self.srv_clb_group)
        self.srv1 = self.create_service(SetBool, '/bring_trash_table', self.table_srv_bring_clb, callback_group=self.srv_clb_group)
        
        
        # Create Twist publisher
        self.speed_pub = self.create_publisher(Twist, '/turtlebot_5/cmd_vel', 3)
        # Create table elevator pubishers
        self.table_up_pub = self.create_publisher(String, '/elevator_up', 3)
        self.table_down_pub = self.create_publisher(String, '/elevator_down', 3)


        self.home_pos = (4.4, -1.2, 0.0)
        self.table_target_pos = (-1.40, 1.0, 1.57)


        self.target_points = [
            (3.5, -0.0, 1.57*2),
            (3.0, -0.15, 1.57*2),
            (2.5, -0.0, 1.57*2),
            (2.0, -0.2, 1.57*2),
            (1.2, -0.05, 1.57*2), # Left side of table in front

            (0.9, -0.05, 1.57*2),
            (0.9, 0.65, 0.0),

            #(0.7, 0.3, 1.57*2),

            (1.2, 0.65, 0.0),
            (2.0, 0.8, 0.0),
            (2.5, 0.65, 0.0),
            (3.0, 0.8, 0.0),
        ]

        self.target_points.append(self.home_pos)

        # TODO set table left point

        self.get_logger().info('Main node started')
    
    def table_srv_bring_clb(self, request, response):

        self.get_logger().info(f"Received service call to bring trash table!")
        self.is_table = False

        # Clear global costmap
        self.navigator.clearGlobalCostmap()

        # # Move down table for safety
        self.table_down_pub.publish(String())

        result = self.go_to_pose(-1.07, -0.5, 1.57*2, search_table=False)

        if result != TaskResult.SUCCEEDED:
            self.get_logger().info(f"Going to point: FAILED {result}")
            #  Go to home point
            result = self.go_to_pose(
                    self.home_pos[0],
                    self.home_pos[1],
                    self.home_pos[2],
                    search_table=False
            )
            response.success = True  # Finished without errors
            response.message = 'Could not find way to the table'
            return response
        
        result = self.go_to_pose(-1.40, 0.1, 1.57, search_table=True)
        if not self.is_table:
            #  Go to home point
            result = self.go_to_pose(
                    self.home_pos[0],
                    self.home_pos[1],
                    self.home_pos[2],
                    search_table=False
            )
            response.success = True  # Finished without errors
            response.message = 'Could not find the table'
            return response

        (status, result) = self.aproach_table(dummy_aproach=False)
        print("Calling action to approach table")
        if status != GoalStatus.STATUS_SUCCEEDED :
            self.get_logger().error(f"Action call failes when approached table")

            response.success = False  # Finished with errors
            response.message = 'Robot got lost and ROS failed. Go and check if everything is okay'
            return response
        
        # 0 - success, 1 - table not found, 2 - failed approach
        if result.result:
            if result.result == 1:
                self.get_logger().info(f"Lost table during approach, go home now")
                #  Go to home point
                result = self.go_to_pose(
                        self.home_pos[0],
                        self.home_pos[1],
                        self.home_pos[2],
                        search_table=False
                )
                response.success = True  # Finished without errors
                response.message = 'Lost table during approach, please check table orientation'
                return response
            
            if result.result == 2:
                self.get_logger().error(f"Robot is stucked under the table")

                response.success = False  # Finished with errors
                response.message = 'Robot is stucked under the table. Go and help!'
                return response
    
        self.get_logger().info(f"action server finished with robot uinder the table with result {result.result}")
        self.table_up_pub.publish(String())

        self.create_rate(1 / 3.0).sleep()  # Sleep 3 sec


        #  Go to unloading point
        result = self.go_to_pose(
                0.5,
                0.5,
                0.0,
                search_table=False
        )
        if result != TaskResult.SUCCEEDED:
            self.get_logger().error(f"Robot with table did not reached unloading point")

            response.success = False  # Finished with errors
            response.message = 'Robot with table did not reached unloading point! Go and help'
            return response
        

        response.success = True  # Finished with errors
        response.message = 'Great job!'
        return response
    

    def table_sub_clb(self, msg):
        self.is_table = msg.data

    # def timer_clb(self):
    #     self.timer.cancel()

    def table_srv_clb(self, request, response):
        
        self.get_logger().info(f"Received service call to move trash table!")

        self.is_table = False

        # Clear global costmap
        self.navigator.clearGlobalCostmap()

        # # Move down table for safety
        self.table_down_pub.publish(String())

        self.action_status = False

        # Step 1 - go to known location where table might be

        for target_point in self.target_points:
            result = self.go_to_pose(target_point[0], target_point[1], target_point[2], search_table=True)
            if self.is_table:
                break
            if result != TaskResult.SUCCEEDED:
                self.get_logger().info(f"Going to point: FAILED {result}, but continue")
        
        if not self.is_table:
            self.get_logger().info("Couldnot find any table during search")
            response.success = True  # Finished without errors
            response.message = 'Couldnot find any table during search'
            return response


        # Step 2 - try to approach table
        (status, result) = self.aproach_table(dummy_aproach=False)
        print("Calling action to approach table")
        if status != GoalStatus.STATUS_SUCCEEDED :
            self.get_logger().error(f"Action call failes when approached table")

            response.success = False  # Finished with errors
            response.message = 'Robot got lost and ROS failed. Go and check if everything is okay'
            return response
        
        # 0 - success, 1 - table not found, 2 - failed approach
        if result.result:
            if result.result == 1:
                self.get_logger().info(f"Lost table during approach, go home now")
                #  Go to home point
                result = self.go_to_pose(
                        self.home_pos[0],
                        self.home_pos[1],
                        self.home_pos[2],
                        search_table=False
                )
                response.success = True  # Finished without errors
                response.message = 'Lost table during approach, please check table orientation'
                return response
            
            if result.result == 2:
                self.get_logger().error(f"Robot is stucked under the table")

                response.success = False  # Finished with errors
                response.message = 'Robot is stucked under the table. Go and help!'
                return response


        self.get_logger().info(f"action server finished with robot uinder the table with result {result.result}")
        self.table_up_pub.publish(String())

        self.create_rate(1 / 3.0).sleep()  # Sleep 3 sec


        self.move_robot(2.0, -0.2)  # Time, speed

        #  Go to unloading point
        result = self.go_to_pose(
                self.table_target_pos[0],
                self.table_target_pos[1],
                self.table_target_pos[2],
                search_table=False
        )
        if result != TaskResult.SUCCEEDED:
            self.get_logger().error(f"Robot with table did not reached unloading point")

            response.success = False  # Finished with errors
            response.message = 'Robot with table did not reached unloading point! Go and help'
            return response
        
        # Move down table
        self.table_down_pub.publish(String())
        self.create_rate(1 / 3.0).sleep()  # Sleep 3 sec

        self.move_robot(4.0, -0.2)  # Time, speed

        # Turn robot as well
        result = self.rotate_robot(1.57*2)
        if not result:
            pass
            return
        
        #  Go to home point
        result = self.go_to_pose(
                self.home_pos[0],
                self.home_pos[1],
                self.home_pos[2],
                search_table=False
        )
        if result != TaskResult.SUCCEEDED:
            self.get_logger().info(f"Robot lost way his way to home!")

            response.success = True  # Finished with errors
            response.message = 'Robot lost way his way to home!'
            return response
        
        self.get_logger().info(f"Robot delivered table and got back home!")

        response.success = True  # Finished with errors
        response.message = 'Robot delivered table and got back home!'
        return response

       
    def get_pose_stamped(self,  x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        q = quaternion_from_euler(0, 0, yaw)

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

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
    
    def go_to_pose(self, x, y, yaw, search_table=False):
        pose = self.get_pose_stamped(x, y, yaw)

        self.get_logger().info(f'Received request for item picking at {pose.pose.position}.')
        self.navigator.goToPose(pose)

        # Do something during our route
        # (e.x. queue up future tasks or detect person for fine-tuned positioning)
        # Simply print information for workers on the robot's ETA for the demonstation
        i = 0
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()

            if search_table:
                if self.is_table:
                    self.navigator.cancelTask()
                    while not self.navigator.isTaskComplete():
                        self.create_rate(1.0).sleep() # Sleep 2sec

                    if not self.is_table:
                        self.get_logger().info("Continue started task")
                        self.navigator.goToPose(pose)
                    else:
                        self.get_logger().info("There is actually table")


            if feedback and i % 100 == 0:
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
    
    def move_robot(self, timeout, speed) -> bool:
        msg = Twist()
        msg.linear.x = speed
        time = 0
        rate = 10
        while time < timeout:
            time += 1/rate
            self.speed_pub.publish(msg)
            self.create_rate(rate).sleep()
    

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

