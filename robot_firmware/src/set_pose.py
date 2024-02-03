import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import SetPose

class SetInitialPose(Node):
    def __init__(self):
        super().__init__('set_initial_pose')
        self.client = self.create_client(SetPose, 'set_pose')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = SetPose.Request()

    def set_initial_pose(self, x, y, theta):
        self.req.pose.position.x = x
        self.req.pose.position.y = y
        self.req.pose.orientation.z = theta
        self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    set_initial_pose = SetInitialPose()
    set_initial_pose.set_initial_pose(0.0, 0.0, 0.0)
    rclpy.spin(set_initial_pose)
    set_initial_pose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()