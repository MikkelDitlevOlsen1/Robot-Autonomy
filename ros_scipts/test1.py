import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
import tf2_ros
#import open3d as o3d

class lidarprocessor(Node):

    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)

        #self.subscription  # prevent unused variable warning

        # transform broadcaster
        #self.tf_broadcaster = tf2_ros.transformbroadcaster(self)
        
        # initial robot pose (x, y, yaw)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        

    def listener_callback(self, msg):
        mapdata=msg.info
        res=mapdata.resolution

        self.get_logger().info(f'res {res}')
        
def main(args=None):
    rclpy.init(args=args)
    lidar_processor = lidarprocessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()