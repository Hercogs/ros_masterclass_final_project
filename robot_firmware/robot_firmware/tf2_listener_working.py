
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TransformListener(Node):
    def __init__(self):
        super().__init__('transform_listener')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_latest_transform(self, target_frame, source_frame):
        try:
            # Get the latest transform from the buffer
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(str(e))
            return None

def main(args=None):
    rclpy.init(args=args)
    transform_listener = TransformListener()

    # Specify the target and source frames
    target_frame = 'robot_base_link'
    source_frame = 'pre_table_frame'

    while rclpy.ok():
        # Get the latest transform
        latest_transform = transform_listener.get_latest_transform(target_frame, source_frame)
        if latest_transform:
            # Print the transform
            print(latest_transform.transform.translation)
        rclpy.spin_once(transform_listener)

    transform_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
