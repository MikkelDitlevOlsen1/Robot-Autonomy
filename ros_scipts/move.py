import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleBotMover(Node):
    def __init__(self):
        super().__init__('turtlebot_mover')

        # Create a publisher for the /cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to publish velocity at a fixed interval (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_velocity)

        # Set movement parameters
        self.linear_velocity = 0.05  # m/s
        self.angular_velocity = 0.0  # rad/s (set >0 for rotation)

    def publish_velocity(self):
        """ Publishes constant velocity to move the TurtleBot """
        msg = Twist()
        msg.linear.x = self.linear_velocity  # Forward movement
        msg.angular.z = self.angular_velocity  # Rotation

        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Publishing linear: {self.linear_velocity} m/s, angular: {self.angular_velocity} rad/s")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
