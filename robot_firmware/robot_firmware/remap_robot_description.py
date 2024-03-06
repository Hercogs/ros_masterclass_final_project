import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, DurabilityPolicy

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        custom_qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.subscription = self.create_subscription(
            String,
            '/rb1_robot/robot_description',
            self.listener_callback,
            custom_qos_profile
        )

        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(String, '/robot_description', custom_qos_profile)

    def listener_callback(self, msg):

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()