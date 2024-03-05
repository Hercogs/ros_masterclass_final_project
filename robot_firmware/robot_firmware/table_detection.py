
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math, time
from statistics import mean

from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from tf_transformations  import quaternion_from_euler
import tf2_ros
from std_msgs.msg import Bool

from rclpy.action import ActionServer

from robot_firmware_interfaces.action import AproachTable  # import the action message
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

#  colcon build --packages-select robot_firmware --symlink-install

# ros2 run robot_firmware table_detection

# ros2 action send_goal /aproach_table robot_firmware_interfaces/action/AproachTable "dummy_aproach: true"

# ros2 launch path_planner_server pathplanner.launch.py
# Radius 0.36

class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')
        self.laser_scan_subscription = self.create_subscription(
            LaserScan,
            '/turtlebot_5/scan',
            self.scan_callback,
            3
        )
        self.laser_scan_subscription  # prevent unused variable warning
        self.br = tf2_ros.TransformBroadcaster(self)

        # Create Twist publisher
        self.speed_pub = self.create_publisher(Twist, '/turtlebot_5/cmd_vel', 3)
         # Create table publisher
        self.table_pub = self.create_publisher(Bool, '/table', 3)

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



        self.publish_table_tf = True  # defult False
        self.is_table = False
        self.maybe_table = False
    

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

        i = 0
        while not self.is_table:
            i += 1
            self.create_rate(10.0).sleep()
            if self.is_table:
                break


        if not self.is_table:
            print("Table not found")
            result.result = 1
            result.msg = "Table not found"
            goal_handle.succeed()
            self.publish_table_tf = False
            return result

        # try to reach pre_table_link
        target_frame = 'turtlebot_5_base_link'
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

            #print(f'Dst: {distance:.2f}, angle: {angle:.2f}')

            if abs(angle) > 0.20: # 13 degree
                msg.linear.x = 0.0
                msg.angular.z = 0.25 * angle / abs(angle)
            else:
                msg.linear.x = scale_forward_speed
                sign = angle / abs(angle)
                msg.angular.z = sign * scale_rotation * abs(angle)

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

        target_frame = 'turtlebot_5_base_link'
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
        self.create_rate(10.0).sleep()

        # CHeck if there is still table
        i = 0
        while not self.is_table and i < 10:
            i += 1
            self.create_rate(10.0).sleep()
            if self.is_table:
                break

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

        target_frame = 'turtlebot_5_base_link'
        ref_frame = 'back_table_frame'

        msg = Twist()
        while(distance > 0.27):
        # Stop publish whgen uder, so no mistkaes
            if distance < 0.8:
                self.publish_table_tf = False
            x1, y1 = self.read_tf(target_frame=target_frame, ref_frame=ref_frame)
            if x1 == None:
                continue
            distance = math.sqrt(x1**2 + y1**2)
            angle = math.atan2(y1, x1)

            #print(f'Dst: {distance:.2f}, angle: {angle:.2f}')

            if abs(angle) > 0.20: # 13 degree
                msg.linear.x = 0.0
                msg.angular.z = 0.20 * angle / abs(angle)
            else:
                msg.linear.x = scale_forward_speed
                sign = angle / abs(angle)
                msg.angular.z = sign * scale_rotation * abs(angle)

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

        if not self.maybe_table:
            return

        # Is table 
        frame_name = [f"leg_{i}" for i in range(4)]

        zone_y = 1.2
        tabe_msg = Bool()
        table_wrong = False
        
        check_frames = ["leg_0", "leg_1", "leg_2", "leg_3", "pre_table_frame"]
        for frame in frame_name:
            x, y = self.read_tf(target_frame="map", ref_frame=frame)
            if y == None:
                return

            if y > zone_y:
                #print(f"Not table or not allowed psoition")
                table_wrong = True
                break
        
        if table_wrong:
            tabe_msg.data = False
            self.is_table = False
            self.table_pub.publish(tabe_msg)
            return
        else:
            tabe_msg.data = True
            self.is_table = True
            self.table_pub.publish(tabe_msg)



        # Get first leg pos
        x1, y1 = self.read_tf(target_frame="turtlebot_5_odom", ref_frame="leg_0") #map
        x2, y2 = self.read_tf(target_frame="turtlebot_5_odom", ref_frame="leg_1") #map

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
        self.publish_tf(x_mid, y_mid, theta=slope, parent_frame = "turtlebot_5_odom", frame_name="front_table_frame") #map

        # Get second leg pos
        x1, y1 = self.read_tf(target_frame="turtlebot_5_odom", ref_frame="leg_2") #map
        x2, y2 = self.read_tf(target_frame="turtlebot_5_odom", ref_frame="leg_3") #map

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
        self.publish_tf(x_mid, y_mid, theta=slope, parent_frame = "turtlebot_5_odom", frame_name="back_table_frame") #map

        # Publish pre table frame
        x1, y1 = self.read_tf(target_frame="front_table_frame", ref_frame="turtlebot_5_base_link")
        if x1 == None:
            return
        # Publish extra frame for robot before going under the table
        d_y = 0.4
        y = -d_y if y1 <= 0 else d_y
        
        self.publish_tf(0.0, y, theta=0.0, parent_frame = "front_table_frame", frame_name="pre_table_frame")

        # Check if table is allowd zones
        # TODO make it cleaner





    def scan_callback(self, msg):
        self.publish_table_tf = True

        if not self.publish_table_tf:
            self.is_table = False
            return
        # Get the range values from the laser scan message
        ranges = msg.ranges

        # Define the minimum and maximum distances for table legs
        min_range = 0.22  # Minimum range value for table detection
        max_range = 1.0  # Maximum range value for table detection

        max_two_point_dist = 0.05  # Maximum distance between 2 laser reading to be in 1 group
        min_group_size = 5

        table_leg_size = 0.9

        table_x_size = 0.5
        table_y_size = 0.5
        table_diagnol = math.sqrt(table_x_size**2 + table_y_size**2)
        table_size_tolerance = 0.05

        # Reverse 
        prev_range = msg.ranges[0]
        scans_too_big = 0

                # Filter parameters
        min_distance = 0.22  # Minimum distance to keep
        max_distance = 1.5  # Maximum distance to keep

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
        
        for i, group in enumerate(filtered_groups):
            pass
            #self.get_logger().info(f'Group {i+1}: {len(group)}, start idx: {group[0][1]*180/3.14:.2f}, end idx: {group[-1][1]*180/3.14:.2f},dist: {mean(list(zip(*group))[0])}')

            mean_dist = mean(list(zip(*group))[0])
            mean_angle = mean(list(zip(*group))[1])

            x = mean_dist * math.cos(mean_angle)
            y = mean_dist* math.sin(mean_angle)

            #self.publish_tf(x, y, frame_name=f"leg_{i}")
        
        

        ### NEW METHOD ###
        class Leg:
            def __init__(self, x=0.0, y=0.0):
                self.x = x
                self.y = y
        
        all_legs = []
        real_legs = []

        # Get x,y for every leg
        for i, group in enumerate(filtered_groups):
            distance = mean(list(zip(*group))[0])
            angle = mean(list(zip(*group))[1])
            leg = Leg(
                x = distance * math.cos(angle),
                y = distance * math.sin(angle)
            )
            all_legs.append(leg)

        dst_btw_legs = []
        table = False
        for i, leg in enumerate(all_legs):
            dst_btw_legs.clear()
            for j, leg_next in enumerate(all_legs):
                if j == i:
                    dst_btw_legs.append(0.0)
                    continue                
                # Calculate the distance between the Cartesian coordinates
                distance = math.sqrt((leg.x - leg_next.x)**2 + (leg.y - leg_next.y)**2)
                dst_btw_legs.append(distance)
            
            leg_x = 0
            leg_y = 0
            leg_middle = 0
            leg_idx = []
            table = False
            # Analyze distances
            for j, d in enumerate(dst_btw_legs):
                x_size = abs(d - table_x_size)
                y_size = abs(d - table_y_size)
                diagnol_size = abs(d - table_diagnol)

                if x_size < table_size_tolerance and \
                        y_size < table_size_tolerance:
                    if  not leg_x: #x_size < y_size and
                        #print(f"leg_x 1: {x_size}")
                        leg_x = j
                        leg_idx.append(j)
                    elif not leg_y:
                        #print(f"leg_y 1: {y_size}")
                        leg_y = j
                        leg_idx.append(j)
                elif x_size < table_size_tolerance and not leg_x:
                    #print(f"leg_x 2: {x_size}")
                    leg_x = j
                    leg_idx.append(j)
                elif y_size < table_size_tolerance and not leg_y:
                    #print(f"leg_y 2: {y_size}")
                    leg_y = j
                    leg_idx.append(j)   
                if diagnol_size < table_size_tolerance and \
                        (leg_x or leg_y):
                    #print("leg_mid")
                    if leg_x and leg_y:
                        #self.get_logger().info(f'Both x, y legss found between midle')
                        break
                    target_leg_idx = leg_x if leg_x else leg_y

                    # Calculate distance from traget to possible midl
                    target_leg = all_legs[target_leg_idx]
                    xy_leg = all_legs[j]
                    distance = math.sqrt((target_leg.x - xy_leg.x)**2 + (target_leg.y - xy_leg.y)**2)
                    d_err = abs(distance - table_x_size)
                    if d_err < table_size_tolerance:
                        leg_middle = j
                        leg_idx.append(j)
                if leg_x and  leg_y and leg_middle:
                    real_legs.clear()
                    real_legs.append(all_legs[i])  # Current leg
                    real_legs.append(all_legs[leg_x]) 
                    real_legs.append(all_legs[leg_y]) 
                    real_legs.append(all_legs[leg_middle])
                    real_legs.sort(key=lambda leg: math.sqrt(leg.x**2 + leg.y**2))

                    d1 = math.sqrt((real_legs[0].x - real_legs[1].x)**2 + (real_legs[0].y - real_legs[1].y)**2)
                    d2 = math.sqrt((real_legs[1].x - real_legs[2].x)**2 + (real_legs[1].y - real_legs[2].y)**2)
                    d3 = math.sqrt((real_legs[2].x - real_legs[3].x)**2 + (real_legs[2].y - real_legs[3].y)**2)
                    d4 = math.sqrt((real_legs[3].x - real_legs[0].x)**2 + (real_legs[3].y - real_legs[0].y)**2)
                    d14 = d1+d2+d3+d4
                    #print(d1, d2, d3,d4, d14)

                    if  abs(table_x_size - d1) < table_size_tolerance and \
                        abs(table_diagnol - d2) < table_size_tolerance and \
                        abs(table_x_size - d3) < table_size_tolerance and \
                        abs(table_diagnol - d4) < table_size_tolerance:
                        #print("table")
                        table = True
                        break
                    #table = True
            
            if table:
                break
            
        if table:
            #self.get_logger().info("TABLE")
            # real_legs.clear()
            # real_legs.append(all_legs[i])  # Current leg
            # real_legs.append(all_legs[leg_x]) 
            # real_legs.append(all_legs[leg_y]) 
            # real_legs.append(all_legs[leg_middle])

            # real_legs.sort(key=lambda leg: math.sqrt(leg.x**2 + leg.y**2))

            for i, leg in enumerate(real_legs):
                self.publish_tf(leg.x, leg.y, frame_name=f"leg_{i}")

            self.maybe_table = True
        else:
            #self.get_logger().info("NO TABLE")
            self.maybe_table = False
            pass
        
        tabe_msg = Bool()
        tabe_msg.data = self.is_table
        #self.table_pub.publish(tabe_msg)


        return


    
    def publish_tf(self, x, y, theta=0, parent_frame = "turtlebot_5_laser_link", frame_name = "table_frame"):
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


    def read_tf(self, target_frame, ref_frame, fixed_frame="map"):
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
