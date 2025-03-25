import rclpy
from rclpy.node import Node
import numpy as np
from nav2_msgs.msg import ParticleCloud
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
#import tf_transformations
import tf2_ros
#import open3d as o3d

class lidarprocessor(Node):

    def __init__(self):
        super().__init__('lidar_processor')
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT  # Set to BEST_EFFORT

        self.subscription = self.create_subscription(
            ParticleCloud,
            '/particle_cloud',
            self.listener_callback,
        qos_profile  # Use the updated QoS profile
        )

        

    def listener_callback(self, msg : ParticleCloud):
        self.get_logger().info(f"Number of particles: {len(msg.particles)}")
        
def main(args=None):
    rclpy.init(args=args)
    lidar_processor = lidarprocessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()