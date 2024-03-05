#! /usr/bin/env python3

import rclpy
from rclpy.action import ActionServer
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

# ros2 run robot_firmware main_node --ros-args -p use_sim_time:=true

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
    
         # Create table subscriber
        self.table_sub = self.create_subscription(Bool, '/table', self.table_sub_clb, 3)
        self.is_table = False

        # self.get_logger().info('Initializing robot pose...')
        # self.set_initial_position()
        
        
        # Create Twist publisher
        self.speed_pub = self.create_publisher(Twist, '/turtlebot_5/cmd_vel', 3)
        # Create table elevator pubishers
        self.table_up_pub = self.create_publisher(String, '/elevator_up', 3)
        self.table_down_pub = self.create_publisher(String, '/elevator_down', 3)


        self.home_pos = (4.4, -1.2, 1.57)
        self.table_target_pos = (-1.25, 1.0, 1.57)


        self.target_points = [
            (3.5, -0.0, 1.57*2),
            (3.0, -0.1, 1.57*2),
            (2.5, -0.2, 1.57*2),
            (2.0, -0.2, 1.57*2),
            (1.2, -0.1, 1.57*2), # Left side of table in front

            (1.2, 0.7, 0.0),
            (2.0, 0.8, 0.0),
            (2.5, 0.7, 0.0),
            (3.0, 0.8, 0.0),
        ]

        self.target_points.append(self.home_pos)

        # TODO set table left point

        self.get_logger().info('Main node started')
    

    def table_sub_clb(self, msg):
        self.is_table = msg.data


    def timer_clb(self):
        self.timer.cancel()

        # # Move down table for safety
        self.table_down_pub.publish(String())

        self.action_status = False

        # Step 1 - go to known location where table might be

        for target_point in self.target_points:
            result = self.go_to_pose(target_point[0], target_point[1], target_point[2], search_table=True)
            if self.is_table:
                break
            if result != TaskResult.SUCCEEDED:
                print(f"Going to point: FAILED {result}, but continue")
        
        if not self.is_table:
            print("Couldnot find any table")
            return


        # Step 2 - trye to approach table
        (status, result) = self.aproach_table(dummy_aproach=False)
        print("Calling action to approach table")
        if status != GoalStatus.STATUS_SUCCEEDED :
            print(f"action client failed: {status}")
            # TODO process failed task
            return
        
        # 0 - success, 1 - table not found, 2 - failed approach
        if result.result:
            print(f"Action cliented returned: {result.result}, {result.msg}")
            # TODO process failed task -  go home
            return

        print(f"action server finished with robot uinder the table with result {result.result}")
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
            print(f"Going to table target: FAILED {result}, but continue")
            # TODO
            return
        
        # Move down table
        self.table_down_pub.publish(String())
        self.create_rate(1 / 3.0).sleep()  # Sleep 3 sec

        self.move_robot(4.0, -0.2)  # Time, speed

        # Turn robot as well
        result = self.rotate_robot(1.57*2)
        if not result:
            print(f"Could not turn the robot")
            # TODO
            return
        
        #  Go to home point
        result = self.go_to_pose(
                self.home_pos[0],
                self.home_pos[1],
                self.home_pos[2],
                search_table=False
        )
        if result != TaskResult.SUCCEEDED:
            print(f"Going to table target: FAILED {result}, but continue")
            # TODO
            return
        
        print("HOMEEE")



        # Step 3 - Rotate robot
        # result = self.rotate_robot(yaw=table_loc[3])
        # if not result:
        #     print(f"Step 3 - Rotate robot: FAILED")
        #     # TODO process failed task
        #     return
        # self.create_rate(1/(5*abs(table_loc[3]))).sleep()
        # print(f"Step 3 - Rotate robot: SUCCESS")
        
        # Step 4 - Lift elevator
        # Step 4.1 - Change footprint


        # Step 5 - Move table to back room


        # Step 6 - Down elevator
        # Step 6.1 - Change footprint


        # Step 7 - Get out of table


        # Step 7 - Go to home position  
        # home_loc = home_position['home_1']
        # result = self.go_to_pose(home_loc[0], home_loc[1], home_loc[2])
        # if result != TaskResult.SUCCEEDED:
        #     print(f"Step 7 - go to home position: FAILED {result}")
        #     # TODO process failed task
        #     return
        # print("Step 7 - go to home positio: DONE")



        
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

        print(f'Received request for item picking at {pose.pose}.')
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
                        print("Continue started task")
                        self.navigator.goToPose(pose)
                    else:
                        print("There is actually table")


            if feedback and i % 50 == 0:
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


