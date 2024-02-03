
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math, time
from statistics import mean

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from tf_transformations  import quaternion_from_euler
import tf2_ros

from rclpy.action import ActionServer

from robot_firmware_interfaces.action import AproachTable  # import the action message
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ros2 run robot_firmware table_detection --ros-args -p use_sim_time:=true

# ros2 action send_goal /aproach_table robot_firmware_interfaces/action/AproachTable "dummy_aproach: true"


class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')
        self.laser_scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            3
        )
        self.laser_scan_subscription  # prevent unused variable warning
        self.br = tf2_ros.TransformBroadcaster(self)

        # Create Twist publisher
        self.speed_pub = self.create_publisher(Twist, 'diffbot_base_controller/cmd_vel_unstamped', 3)

        # Create tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create action server for approaching table
        self.action_server = ActionServer(
            self,
            AproachTable,
            'aproach_table',
            self.execute_action_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Timer to publish tf to approach table
        self.timer_tf = self.create_timer(0.1, self.timer_tf_clb, callback_group=MutuallyExclusiveCallbackGroup())



        self.publish_table_tf = False  # defult False
        self.is_table = False
    

    def execute_action_callback(self, goal_handle):
        # Enable table tf publishing
        self.publish_table_tf = True

        self.create_rate(1/2.0).sleep()  # Sleep 2 sec

        # 0 - success, 1 - table not found, 2 - failed approach
        result = AproachTable.Result()

        # self.publish_table_tf = False
        # goal_handle.succeed()
        # return result

        dummy_aproach = goal_handle.request.dummy_aproach

        if not self.is_table:
            print("Table not found")
            result.result = 1
            result.msg = "Table not found"
            goal_handle.succeed()
            self.publish_table_tf = False
            return result

        # try to reach pre_table_link
        target_frame = 'robot_base_link'
        ref_frame = 'pre_table_frame'
        scale_forward_speed = 0.15
        scale_rotation = 0.30

        distance = 99.0
        # angle = 0.0

        msg = Twist()
        while(distance > 0.03):
            x1, y1 = self.read_tf(target_frame=target_frame, ref_frame=ref_frame)
            if x1 == None:
                continue
            distance = math.sqrt(x1**2 + y1**2)
            angle = math.atan2(y1, x1)

            print(f'Dst: {distance:.2f}, angle: {angle:.2f}')

            if abs(angle) > 0.20: # 13 degree
                msg.linear.x = 0.0
                msg.angular.z = 0.20 * angle / abs(angle)
            else:
                msg.linear.x = scale_forward_speed
                msg.angular.z = scale_rotation * angle / abs(angle)

            self.speed_pub.publish(msg)

            time.sleep(0.1)
        
        print("Reached pre table")
        # Stop robot
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.speed_pub.publish(msg)
        #self.create_rate(1.0).sleep()


        # Turn to "back_table_frame"
        #distance = 99.0
        angle = 99.0

        target_frame = 'robot_base_link'
        ref_frame = 'back_table_frame'

        msg = Twist()
        while(abs(angle) > 0.087): # 5 degrees
            x1, y1 = self.read_tf(target_frame=target_frame, ref_frame=ref_frame)
            if x1 == None:
                continue
            # distance = math.sqrt(x1**2 + y1**2)
            angle = math.atan2(y1, x1)

            # print(f'Dst: {distance:.2f}, angle: {angle:.2f}')

            msg.angular.z = 0.20 * angle / abs(angle)

            self.speed_pub.publish(msg)

            time.sleep(0.1)
        
        print("Turned to back table link")
        # Stop robot
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.speed_pub.publish(msg)
        #self.create_rate(1.0).sleep()

        # CHeck if there is still table
        if not self.is_table:
            print("Table not found after in fro of table")
            result.result = 1
            result.msg = "Table not found"
            goal_handle.succeed()
            self.publish_table_tf = False
            return result

        # Table is found
        if dummy_aproach:
            print("Table on robot - dummy approach")
            result.result = 0
            result.msg = "Table on robot - dummy approach"
            goal_handle.succeed()
            self.publish_table_tf = False
            return result

        # Go to "back_table_frame"
        distance = 99.0
        #angle = 99.0

        target_frame = 'robot_base_link'
        ref_frame = 'back_table_frame'

        msg = Twist()
        while(distance > 0.4):
        # Stop publish whgen uder, so no mistkaes
            if distance < 0.8:
                self.publish_table_tf = False
            x1, y1 = self.read_tf(target_frame=target_frame, ref_frame=ref_frame)
            if x1 == None:
                continue
            distance = math.sqrt(x1**2 + y1**2)
            angle = math.atan2(y1, x1)

            print(f'Dst: {distance:.2f}, angle: {angle:.2f}')

            if abs(angle) > 0.20: # 13 degree
                msg.linear.x = 0.0
                msg.angular.z = 0.20 * angle / abs(angle)
            else:
                msg.linear.x = scale_forward_speed
                msg.angular.z = scale_rotation * angle / abs(angle)

            self.speed_pub.publish(msg)

            time.sleep(0.1)


        print("Robot is under table")

        
        # self.get_logger().error(f"Dummy approach is not implemented yet")
        # Stop robot
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.speed_pub.publish(msg)

        result.result = 0
        result.msg = "Robot is under table"
        goal_handle.succeed()
        self.publish_table_tf = False
        return result


    def timer_tf_clb(self):
        # NOTE - timer execution might be longer than callback interval

        if not self.is_table:
            return

        # Is table 
        frame_name = [f"leg_{i}" for i in range(4)]

        # Get first leg pos
        x1, y1 = self.read_tf(target_frame="odom", ref_frame="leg_0") #map
        x2, y2 = self.read_tf(target_frame="odom", ref_frame="leg_1") #map

        if x1 == None or x2 == None:
            return
        
        dy = y2 - y1
        dx = x2 - x1
        if dy == 0:
            dy = 1e-6
        if dx == 0:
            dx = 1e-6
        
        x_mid = (x1 + x2)/2
        y_mid = (y1 + y2)/2
        slope = math.atan2(dy, dx)
        self.publish_tf(x_mid, y_mid, theta=slope, parent_frame = "odom", frame_name="front_table_frame") #map

        # Get second leg pos
        x1, y1 = self.read_tf(target_frame="odom", ref_frame="leg_2") #map
        x2, y2 = self.read_tf(target_frame="odom", ref_frame="leg_3") #map

        if x1 == None or x2 == None:
            return -1
        
        dy = y2 - y1
        dx = x2 - x1
        if dy == 0:
            dy = 1e-6
        if dx == 0:
            dx = 1e-6
        
        x_mid = (x1 + x2)/2
        y_mid = (y1 + y2)/2
        slope = math.atan2(dy, dx)
        self.publish_tf(x_mid, y_mid, theta=slope, parent_frame = "odom", frame_name="back_table_frame") #map

        # Publish pre table frame
        x1, y1 = self.read_tf(target_frame="front_table_frame", ref_frame="robot_base_link")
        if x1 == None:
            return
        # Publish extra frame for robot before going under the table
        d_y = 0.4
        y = -d_y if y1 <= 0 else d_y
        
        self.publish_tf(0.0, y, theta=0.0, parent_frame = "front_table_frame", frame_name="pre_table_frame")




    def scan_callback(self, msg):

        if not self.publish_table_tf:
            self.is_table = False
            return
        # Get the range values from the laser scan message
        ranges = msg.ranges

        # Define the minimum and maximum distances for table legs
        min_range = 0.1  # Minimum range value for table detection
        max_range = 2.5  # Maximum range value for table detection

        max_two_point_dist = 0.07  # Maximum distance between 2 laser reading to be in 1 group
        min_group_size = 4

        table_leg_size = 0.22

        table_x_size = 0.66
        table_y_size = 0.75
        table_diagnol = math.sqrt(table_x_size**2 + table_y_size**2)
        table_size_tolerance = 0.1

        #group = []
        prev_range = msg.ranges[0]
        scans_too_big = 0

                # Filter parameters
        min_distance = 0.1  # Minimum distance to keep
        max_distance = 2.5  # Maximum distance to keep

        # Create a new LaserScan message for filtered data
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max

        # Filter the ranges based on distance
        filtered_ranges = []
        for range in msg.ranges:
            if min_distance <= range <= max_distance:
                filtered_ranges.append(range)
            else:
                filtered_ranges.append(0.0)  # Set invalid ranges to 0.0

        # Update the filtered ranges in the new LaserScan message
        filtered_msg.ranges = filtered_ranges


        # Split in groups or objects
        splitted_groups = []
        self.split_in_groups(filtered_msg, splitted_groups, max_two_point_dist)


        # Remove too small groups/noise
        filtered_groups_by_size = []
        self.filter_groups_by_size(splitted_groups, filtered_groups_by_size, min_group_size)
        

        # Filter groups by shape
        filtered_groups = []
        for i, group in enumerate(filtered_groups_by_size):
            #print(list(zip(*group)))
            if len(list(zip(*group))) == 0:
                continue
            min_dist = min(list(zip(*group))[0])
            max_dist = max(list(zip(*group))[0])
            delta_dist = abs(max_dist - min_dist)
            start_angle = list(zip(*group))[1][0]
            start_dist = list(zip(*group))[0][0]
            end_angle = list(zip(*group))[1][-1]
            end_dist = list(zip(*group))[0][-1]
            dist_between_start_end = self.distance_between_polar_coordinates(start_dist, start_angle, end_dist, end_angle)

            #print(f"Delta {delta_dist:.2f} {dist_between_start_end:.2f}")

            if dist_between_start_end <= table_leg_size and \
                    delta_dist <= table_leg_size:
                filtered_groups.append(group)
        

        table = False
        class TableLeg:
            distance = 0
            angle = 0

        front_left_leg = TableLeg()
        back_left_leg = TableLeg()
        back_right_leg = TableLeg()
        front_right_leg = TableLeg()
    
        if len(filtered_groups) < 4:
            print("Too less groups")

        # Calculate distacne betwen groups
        for i, group in enumerate(filtered_groups):
            # Check if there are at least 3 legs left
            if not len(filtered_groups) - i >= 3:
                #print("NO TABLE")
                break
            if len(list(zip(*group))) == 0:
                continue
            first_leg = TableLeg()
            first_leg.distance = mean(list(zip(*group))[0])
            first_leg.angle = mean(list(zip(*group))[1])
            possible_table_leg_distances = []
            table_legs = []
            table_legs.append(first_leg)

            for j, group_target in enumerate(filtered_groups):
                if j <= i:
                    continue
                possible_leg = TableLeg()
                possible_leg.distance = mean(list(zip(*group_target))[0])
                possible_leg.angle = mean(list(zip(*group_target))[1])
                dist_between_start_end = self.distance_between_polar_coordinates(
                    first_leg.distance,
                    first_leg.angle,
                    possible_leg.distance,
                    possible_leg.angle)
                possible_table_leg_distances.append(dist_between_start_end)
                table_legs.append(possible_leg)

            
            # Analyze distances
            #print(dst)
            #print([f"{i:.2f}" for i in possible_table_leg_distances])

            #print(len(possible_table_leg_distances), len(table_legs))

            leg_x = False
            leg_y = False
            leg_middle = False
            leg_idx = []
            for i, d in enumerate(possible_table_leg_distances):
                x_size = abs(d - table_x_size)
                y_size = abs(d - table_y_size)
                diagnol_size = abs(d - table_diagnol)

                #print("Table errors:")
                #print(x_size, y_size, diagnol_size)

                if x_size < table_size_tolerance and \
                        y_size < table_size_tolerance:
                    if x_size < y_size and not leg_x:
                        #print(f"leg_x 1: {x_size}")
                        leg_x = True
                        leg_idx.append(i)
                    elif not leg_y:
                        #print(f"leg_y 1: {y_size}")
                        leg_y = True
                        leg_idx.append(i)
                elif x_size < table_size_tolerance and not leg_x:
                    #print(f"leg_x 2: {x_size}")
                    leg_x = True
                    leg_idx.append(i)
                elif y_size < table_size_tolerance and not leg_y:
                    #print(f"leg_y 2: {y_size}")
                    leg_y = True
                    leg_idx.append(i)   
                if diagnol_size < table_size_tolerance and \
                        (leg_x or leg_y):
                    #print("leg_mid")
                    leg_middle = True
                    leg_idx.append(i)
                if leg_x and  leg_y and leg_middle:
                    break
            
            if leg_x and  leg_y and leg_middle:
                table = True
                #print([f"{d:.2f} " for d in possible_table_leg_distances])
                break
        if table:
            #self.get_logger().info("TABLE")
            # info = f"{i}" for i in leg_idx
            #self.get_logger().info("".join(map(str, leg_idx)))
            front_left_leg = table_legs[0]
            back_left_leg = table_legs[leg_idx[0]+1]
            back_right_leg = table_legs[leg_idx[1]+1]
            front_right_leg = table_legs[leg_idx[2]+1]
        else:
            self.get_logger().info("NO TABLE")
        
        if table:
            legs = [front_left_leg, back_left_leg, back_right_leg, front_right_leg]
            # Sort legs by length to robot
            legs.sort(key=lambda x:x.distance)
            #self.get_logger().info("__________")
            legs_x = 0
            legs_y = 0
            for i, leg in enumerate(legs):
                x = leg.distance * math.cos(leg.angle)
                y = leg.distance * math.sin(leg.angle)
                legs_x += x
                legs_y +=y
                self.publish_tf(x, y, frame_name=f"leg_{i}")
                # self.get_logger().info(f"angle: {leg.angle:.2f}")
                # self.get_logger().info(f"dist: {leg.distance:.2f}")


            # mean_angle = mean([i.angle for i in legs])
            # mean_distance = mean([i.distance for i in legs])
            # x = mean_distance * math.cos(mean_angle)
            # y = mean_distance * math.sin(mean_angle)
            
            self.publish_tf(legs_x/4, legs_y/4)

            legs.sort(key=lambda x:x.distance)

            first_leg = legs[0]
            second_leg = legs[1]
            first_leg_x = first_leg.distance * math.cos(first_leg.angle)
            first_leg_y = first_leg.distance * math.sin(first_leg.angle)
            second_leg_x = second_leg.distance * math.cos(second_leg.angle)
            second_leg_y = second_leg.distance * math.sin(second_leg.angle)

            x_mid = (first_leg_x + second_leg_x)/2
            y_mid = (first_leg_y + second_leg_y)/2
            slope = second_leg_y - first_leg_y / second_leg_x - first_leg_x
            #self.publish_tf(x_mid, y_mid, theta=slope, frame_name="pre_table_frame")

            self.is_table = True

        else:
            self.is_table = False


    
    def publish_tf(self, x, y, theta=0, parent_frame = "robot_front_laser_base_link", frame_name = "table_frame"):
        # TF broadcaster
        transfor_msg = TransformStamped()
        # transfor_msg.header.stamp = self.get_clock().now().to_msg()
        transfor_msg.header.stamp = self.get_clock().now().to_msg()
        # xx = int(transfor_msg.header.stamp.sec)
        # self.get_logger().info(f"{transfor_msg.header.stamp}")
        transfor_msg.header.frame_id = parent_frame
        transfor_msg.child_frame_id = frame_name

        # Find middle point
        transfor_msg.transform.translation.x = x
        transfor_msg.transform.translation.y = y
        transfor_msg.transform.translation.z = 0.0

        # Rotation
        q = quaternion_from_euler(0, 0, theta)

        transfor_msg.transform.rotation.x = q[0]
        transfor_msg.transform.rotation.y = q[1]
        transfor_msg.transform.rotation.z = q[2]
        transfor_msg.transform.rotation.w = q[3]

        self.br.sendTransform(transfor_msg)


    def read_tf(self, target_frame, ref_frame, fixed_frame="odom"):
        try:
            t = self.tf_buffer.lookup_transform_full(
                target_frame=target_frame,
                target_time=rclpy.time.Time(),
                source_frame=ref_frame,
                source_time=rclpy.time.Time(),
                fixed_frame=fixed_frame,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(
                            f'Could not transform {target_frame} to {ref_frame}: {ex}')
            return (None, None)

        return (t.transform.translation.x, t.transform.translation.y)
        distance = math.sqrt(
                    t.transform.translation.x ** 2 +
                    t.transform.translation.y ** 2)
        
        angle = math.atan2(
                    t.transform.translation.y,
                    t.transform.translation.x)
        
    
    @staticmethod
    def split_in_groups(msg, groups, max_two_point_dist):
        prev_range = msg.ranges[0]
        prev_idx = msg.angle_min

        max_two_point_dist = max_two_point_dist
        group = []

        # Iterate through the laser scan readings
        for i in range(1, len(msg.ranges)):
            if msg.ranges[i] == 0.0:
                continue
            curr_range = msg.ranges[i]
            curr_idx = msg.angle_min + msg.angle_increment * i

            # Check if the current reading is part of the same object
            d = TableDetectionNode.distance_between_polar_coordinates(
                prev_range,
                prev_idx,
                curr_range,
                curr_idx
            )
            if abs(d) < max_two_point_dist:
                group.append((curr_range, curr_idx))
            else:
                # Add the current group to the list of groups
                if len(group) >= 0:
                    groups.append(group)
            
                # Start a new group with the current reading
                group = [(curr_range, curr_idx)]

            # Update the previous range
            prev_range = curr_range
            prev_idx = curr_idx

        # Add the last group to the list of groups
        if len(group) >= 0:
            groups.append(group)
    
    @staticmethod
    def filter_groups_by_size(in_groups, out_groups, min_group_size):
        # remove too small group by thershold value
        for i, group in enumerate(in_groups):
            if len(group) >= min_group_size:
                out_groups.append(group)
        
    @staticmethod
    def merge_groups_by_size(in_groups, out_groups, max_two_point_dist):
        for i, group in enumerate(in_groups):
            if i == 0:
                out_groups.append(group)
                continue
            start_angle = min(list(zip(*group))[1])
            start_dist = list(zip(*group))[0][0]
            # Debug
            #print(f"Idx {i}, group len: {len(group)}")

            end_angle = max(list(zip(*out_groups[-1]))[1])
            end_dist = list(zip(*out_groups[-1]))[0][-1]
            dist_between_start_end = TableDetectionNode.distance_between_polar_coordinates(start_dist, start_angle, end_dist, end_angle)

            if dist_between_start_end <= max_two_point_dist+0.01:
                #print("Append to prev")

                out_groups[-1].extend(group)
            else:
                out_groups.append(group)

    @staticmethod
    def distance_between_polar_coordinates(r1, theta1, r2, theta2):
        # Convert polar coordinates to Cartesian coordinates
        x1 = r1 * math.cos(theta1)
        y1 = r1 * math.sin(theta1)
        x2 = r2 * math.cos(theta2)
        y2 = r2 * math.sin(theta2)
        
        # Calculate the distance between the Cartesian coordinates
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        return distance





def main(args=None):
    rclpy.init(args=args)
    table_detection_node = TableDetectionNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(table_detection_node)

    executor.spin()
    executor.shutdown()

    table_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
