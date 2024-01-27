
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

        max_two_point_dist = 0.02  # Maximum distance between 2 laser reading to be in 1 group
        min_group_size = 5
        max_missing_reading = 1

        table_leg_size = 0.15

        table_x_size = 0.8
        table_y_size = 1.0

        self.groups.clear()

        group = []
        prev_range = msg.ranges[0]

        scans_too_big = 0

        # Iterate through the laser scan readings
        for i in range(1, len(msg.ranges)):
            curr_range = msg.ranges[i]
            curr_idx = msg.angle_min + msg.angle_increment * i

            # if not min_range <= curr_range <= max_range:
            #     group.clear()
            #     scans_too_big += 1
            #     prev_range = curr_range
            #     continue

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
        print(f"Total {len(self.groups)} groups detected")

        for i, group in enumerate(self.groups):
            self.get_logger().info(f'Group {i+1}: {len(group)}, start idx: {group[0][1]*180/3.14:.2f}, end idx: {group[-1][1]*180/3.14:.2f},dist: {mean(list(zip(*group))[0])}')
         

        filtered_groups = []
        for i, group in enumerate(self.groups):
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

        print(f"Total {len(filtered_groups)} groups detected after filtering v0")

        filtered_groups_v1 = []
        for i, group in enumerate(filtered_groups):
            #print(list(zip(*group)))
            if i == 0:
                filtered_groups_v1.append(group)
                continue
            # min_dist = min(list(zip(*group))[0])
            # max_dist = max(list(zip(*group))[0])
            # delta_dist = abs(max_dist - min_dist)
            start_angle = min(list(zip(*group))[1])
            start_dist = list(zip(*group))[0][0]
            end_angle = max(list(zip(*filtered_groups_v1[-1]))[1])
            end_dist = list(zip(*filtered_groups_v1[-1]))[0][-1]
            dist_between_start_end = self.distance_between_polar_coordinates(start_dist, start_angle, end_dist, end_angle)

            if dist_between_start_end <= max_two_point_dist:
                print("Append to prev")

                filtered_groups_v1[-1].extend(group)
            else:
                filtered_groups_v1.append(group)

        print(f"Total {len(filtered_groups_v1)} groups detected after filtering v1")


        # print(f"Scans too big : {scans_too_big} out of {len(msg.ranges)}")
        for i, group in enumerate(filtered_groups_v1):
            self.get_logger().info(f'Group {i+1}: {len(group)}, start idx: {group[0][1]*180/3.14:.2f}, end idx: {group[-1][1]*180/3.14:.2f},dist: {mean(list(zip(*group))[0])}')
            


        # # Iterate through the range values to find the distances of table legs
        # for i, distance in enumerate(ranges):
        #     if min_range <= distance <= max_range:
        #         # Check if the distance is within the valid range for a table leg
        #         if i < len(ranges) // 2:
        #             # Check if the distance is on the left side of the laser scan
        #             left_leg_distance = min(left_leg_distance, distance)
        #         else:
        #             # Check if the distance is on the right side of the laser scan
        #             right_leg_distance = min(right_leg_distance, distance)

        # # Check if both table legs are detected
        # if left_leg_distance != float('inf') and right_leg_distance != float('inf'):
        #     # Calculate the distance between the table legs
        #     table_width = abs(left_leg_distance - right_leg_distance)

        #     # Check if the table width is within a valid range
        #     if 0.5 <= table_width <= 1.5:
        #         self.get_logger().info('Table detected!')

def main(args=None):
    rclpy.init(args=args)
    table_detection_node = TableDetectionNode()
    rclpy.spin(table_detection_node)
    table_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
