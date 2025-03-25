
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class RobotPositionNode(Node):
    def __init__(self):
        super().__init__('robot_position_node')
        # Create a subscription to the /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # The topic name (change this if your robot uses a different topic name)
            self.odom_callback,
            10  # QoS (Quality of Service) setting
        )
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        # Extract the position from the Odometry message
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        z = position.z  # Usually for 2D robots, z won't change
        self.get_logger().info(f"Robot Position: x = {x}, y = {y}, z = {z}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotPositionNode()

    # Spin the node to keep receiving messages
    rclpy.spin(node)

    # Shutdown the node after use
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()