
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
from statistics import mean

class TableDetectionNode(Node):
    def __init__(self):
        super().__init__('table_detection_node')
        self.laser_scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            5
        )
        self.laser_scan_subscription  # prevent unused variable warning

        self.groups = []
    
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

    def scan_callback(self, msg):
        # Get the range values from the laser scan message
        ranges = msg.ranges

        # print(msg)

        # print("#"*10)
        # print([f"{i:.2f}" for i in ranges])
        # return

        # Define the minimum and maximum distances for table legs
        min_range = 0.2  # Minimum range value for table detection
        max_range = 2.5  # Maximum range value for table detection

        max_two_point_dist = 0.03  # Maximum distance between 2 laser reading to be in 1 group
        min_group_size = 3
        max_missing_reading = 1

        table_leg_size = 0.20

        table_x_size = 0.7
        table_y_size = 0.6
        table_diagnol = math.sqrt(0.7**2 + 0.6**2)
        table_size_tolerance = 0.1

        self.groups.clear()

        group = []
        prev_range = msg.ranges[0]

        scans_too_big = 0

        # Iterate through the laser scan readings
        for i in range(1, len(msg.ranges)):
            curr_range = msg.ranges[i]
            curr_idx = msg.angle_min + msg.angle_increment * i

            # Check if the current reading is part of the same object
            if abs(curr_range - prev_range) < max_two_point_dist:
                group.append((curr_range, curr_idx))
            else:
                # Add the current group to the list of groups
                #if len(group) >= min_group_size:
                self.groups.append(group)
            
                # Start a new group with the current reading
                group = [(curr_range, curr_idx)]

            # Update the previous range
            prev_range = curr_range

        # Add the last group to the list of groups
        if len(group) >= 0:
            self.groups.append(group)

        # Print the groups
        #print(f"Total {len(self.groups)} groups detected")

        for i, group in enumerate(self.groups):
            if len(list(zip(*group))) == 0:
                continue
            #self.get_logger().info(f'Group {i+1}: {len(group)}, start idx: {group[0][1]*180/3.14:.2f}, end idx: {group[-1][1]*180/3.14:.2f},dist: {mean(list(zip(*group))[0])}')
        
        # remove too small group
        big_groups = []
        for i, group in enumerate(self.groups):
            if len(group) >= min_group_size:
                big_groups.append(group)

        #print(f"Total {len(big_groups)} groups detected after removing small ones")

        # Merge close groups
        merge_groups = []
        for i, group in enumerate(big_groups):
            #print(list(zip(*group)))
            if i == 0:
                merge_groups.append(group)
                continue
            start_angle = min(list(zip(*group))[1])
            start_dist = list(zip(*group))[0][0]
            end_angle = max(list(zip(*merge_groups[-1]))[1])
            end_dist = list(zip(*merge_groups[-1]))[0][-1]
            dist_between_start_end = self.distance_between_polar_coordinates(start_dist, start_angle, end_dist, end_angle)

            if dist_between_start_end <= max_two_point_dist+0.01:
                #print("Append to prev")

                merge_groups[-1].extend(group)
            else:
                merge_groups.append(group)

        #print(f"Total {len(merge_groups)} groups detected after merging")

        filtered_groups = []
        for i, group in enumerate(merge_groups):
            #print(list(zip(*group)))
            if len(list(zip(*group))) == 0:
                continue
            min_dist = min(list(zip(*group))[0])
            max_dist = max(list(zip(*group))[0])
            delta_dist = abs(max_dist - min_dist)
            start_angle = min(list(zip(*group))[1])
            start_dist = list(zip(*group))[0][0]
            end_angle = max(list(zip(*group))[1])
            end_dist = list(zip(*group))[0][-1]
            dist_between_start_end = self.distance_between_polar_coordinates(start_dist, start_angle, end_dist, end_angle)

            if dist_between_start_end <= table_leg_size and \
                    len(group) >= min_group_size and \
                    delta_dist <= table_leg_size and \
                    min_dist >= min_range and \
                    max_dist <= max_range:
                filtered_groups.append(group)

        #print(f"Total {len(filtered_groups)} groups detected after filtering v0")


        # for i, group in enumerate(filtered_groups):
        #     self.get_logger().info(f'Group {i+1}: {len(group)}, start idx: {group[0][1]*180/3.14:.2f}, end idx: {group[-1][1]*180/3.14:.2f},dist: {mean(list(zip(*group))[0])}')
            
        table = False
        # Calculate distacne betwen groups
        for i, group in enumerate(filtered_groups):
            # Check if there are at least 3 legs left
            if not len(filtered_groups) - i >= 3:
                #print("NO TABLE")
                break
            if len(list(zip(*group))) == 0:
                continue
            current_group_dist = mean(list(zip(*group))[0])
            current_group_angle = mean(list(zip(*group))[1])
            dst = []

            for j, group_target in enumerate(filtered_groups):
                if j <= i:
                    continue
                target_group_dist = mean(list(zip(*group_target))[0])
                target_group_angle = mean(list(zip(*group_target))[1])
                dist_between_start_end = self.distance_between_polar_coordinates(current_group_dist, current_group_angle, target_group_dist, target_group_angle)
                dst.append(dist_between_start_end)
            
            # Analyze distances
            #print(dst)
            #print([f"{i:.2f}" for i in dst])

            leg_x = False
            leg_y = False
            leg_middle = False
            for d in dst:
                x_size = abs(d - table_x_size)
                y_size = abs(d - table_y_size)
                diagnol_size = abs(d - table_diagnol)
                if x_size < table_size_tolerance and \
                        y_size < table_size_tolerance:
                    if x_size < y_size:
                        #print("leg_x")
                        leg_x = True
                    else:
                        #print("leg_y")
                        leg_y = True
                elif x_size < table_size_tolerance:
                    leg_x = True
                elif y_size < table_size_tolerance:
                    leg_y = True    
                if diagnol_size < table_size_tolerance and \
                        (leg_x or leg_y):
                    #print("leg_mid")
                    leg_middle = True
            
            if leg_x and  leg_y and leg_middle:
                table = True
        if table:
            self.get_logger().info("TABLE")
        else:
            self.get_logger().info("NO TABLE")





def main(args=None):
    rclpy.init(args=args)
    table_detection_node = TableDetectionNode()
    rclpy.spin(table_detection_node)
    table_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
