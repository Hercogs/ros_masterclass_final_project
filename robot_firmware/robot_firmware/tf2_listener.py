import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
import math
from statistics import mean
import time

# ros2 run robot_firmware tf2_listener --ros-args -p use_sim_time:=true

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from tf_transformations  import quaternion_from_euler
import tf2_ros

class TfListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        
        # Create tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create Twist publisher
        self.speed_pub = self.create_publisher(Twist, 'diffbot_base_controller/cmd_vel_unstamped', 3)

        self.timer = self.create_timer(1.0, self.timer_clb, callback_group=MutuallyExclusiveCallbackGroup())
        self.timer1 = self.create_timer(0.1, self.timer_clb1)
    
    def timer_clb1(self):
        pass


    def timer_clb(self):
        self.timer.cancel()
        print("timer clb")

        # Try to find transform
        target_frame = 'robot_base_link'
        ref_frame = 'pre_table_frame'

        # target_frame = 'robot_base_link'
        # ref_frame = 'odom'
    
        scale_forward_speed = 0.1
        scale_rotation = 0.8

        distance = 99.0
        angle = 0.0

        msg = Twist()

        #rclpy.spin_once(self.tf_listener)

        while(distance > 0.1):
            try:
                t = self.tf_buffer.lookup_transform_full(
                    target_frame=target_frame,
                    target_time=rclpy.time.Time(),
                    source_frame=ref_frame,
                    source_time=rclpy.time.Time(),
                    fixed_frame='odom',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
                # t = self.tf_buffer.lookup_transform(
                #     target_frame=target_frame,
                #     source_frame=ref_frame,
                #     time=rclpy.time.Time(), # rclpy.time.Duration(seconds=5.0)
                #     #timeout=rclpy.duration.Duration(seconds=1.1)
                # )
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(
                                f'Could not transform {target_frame} to {ref_frame}: {ex}')
                time.sleep(0.1)
                # TODO
                continue 
            distance = math.sqrt(
                        t.transform.translation.x ** 2 +
                        t.transform.translation.y ** 2)
            
            angle = math.atan2(
                        t.transform.translation.y,
                        t.transform.translation.x)
            
            print(f'Dst: {distance:.2f}, angle: {angle:.2f}')

            if abs(angle) > 0.20: # 13 degree
                msg.linear.x = 0.0
                msg.angular.z = 0.20 * angle / abs(angle)
            else:
                msg.linear.x = 0.1
                msg.angular.z = scale_rotation * 0.20 * angle / abs(angle)

            self.speed_pub.publish(msg)

            time.sleep(0.05)
        

            
        print("Reached pre goal")
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.speed_pub.publish(msg)
        self.create_rate(1.0).sleep()

        distance = 99.0
        ref_frame = 'table_frame'

        while(distance > 0.1):
            try:
                t = self.tf_buffer.lookup_transform_full(
                    target_frame=target_frame,
                    target_time=rclpy.time.Time(),
                    source_frame=ref_frame,
                    source_time=rclpy.time.Time(),
                    fixed_frame='odom',
                    timeout=rclpy.duration.Duration(seconds=1)
                )
            except tf2_ros.TransformException as ex:
                self.get_logger().warn(
                                f'Could not transform {target_frame} to {ref_frame}: {ex}')
                time.sleep(0.1)
                # TODO
                continue 
            distance = math.sqrt(
                        t.transform.translation.x ** 2 +
                        t.transform.translation.y ** 2)
            
            angle = math.atan2(
                        t.transform.translation.y,
                        t.transform.translation.x)
            
            print(f'Dst: {distance:.2f}, angle: {angle:.2f}')
            
            if abs(angle) > 0.20: # 13 degree
                msg.linear.x = 0.0
                msg.angular.z = 0.10 * angle / abs(angle)
            else:
                msg.linear.x = 0.15
                msg.angular.z = scale_rotation * 0.20 * angle / abs(angle)

            self.speed_pub.publish(msg)

            time.sleep(0.05)
            
        print("Reached goal")

            
        self.get_logger().info(f'Finished moving to TF')
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.speed_pub.publish(msg)
        self.create_rate(1.0).sleep()

        return


def main(args=None):
    rclpy.init(args=args)
    table_detection_node = TfListener()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(table_detection_node)

    executor.spin()
    executor.shutdown()

    table_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()