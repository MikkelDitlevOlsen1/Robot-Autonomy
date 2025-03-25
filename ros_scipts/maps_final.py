import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, TransformStamped, Point 
from std_msgs.msg import Header
import random
import tf2_ros
import math


class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher_node')
        self.angles = np.zeros(360) 
        self.ranges = np.zeros(360)
        self.y = 0.0
        self.x = 0.0
        self.w = 0.0
        self.subscription_pos = self.create_subscription(
            Odometry,
            '/odom',  # The topic name (change this if your robot uses a different topic name)
            self.odom_callback,
            10  # QoS (Quality of Service) setting
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

       
        self.subscription_pos  # prevent unused variable warning

                # Define the map size (10x10 matrix as an example)
        self.resolution = 0.1  # resolution of 0.5 meters per grid cell
        self.map_width = int(40*(1//self.resolution))
        self.map_height = int(40*(1//self.resolution))

        # Initialize the map with free cells (value 0)
        self.map = [[-1 for _ in range(self.map_width)] for _ in range(self.map_height)]
        

        # Create a publisher for the /map topic
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Timer to publish map at 0.5 Hz
        self.timer = self.create_timer(2.0, self.publish_map)

        # Create a static transform broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Broadcast a static transform from 'odom' frame to 'map' frame
        self.broadcast_static_transform()

    def odom_callback(self, msg):
        # Extract the position from the Odometry message
        #self.get_logger().info(f"sefl x {self.x}")
        position = msg.pose.pose.position
        self.x = position.x
        self.y = position.y
        
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.w = math.atan2(siny_cosp, cosy_cosp)
        #self.w = math.degrees(self.w)



        #z = position.z  # Usually for 2D robots, z won't change
       # self.get_logger().info(f"Robot Position: x = {self.x}, y = {self.y} ")

    def listener_callback(self, msg):
        #current_scan = msg.ranges #floeat32[]
        #angel_min= msg.angle_min #floeat32
        #angel_max = msg.angle_max #floeat32
        #angel_inclement= msg.angle_increment#floeat32
        self.ranges = np.array(msg.ranges)
        #TODO add robotes own rotation
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(self.ranges))
        #scan_points = np.vstack((x_points, y_points)).t  # nx2 matrix

    def populate_obstacles(self):
        # Define the number of obstacle
        x_org=(-(self.map_width/2)*self.resolution)
        y_org=(-(self.map_height/2)*self.resolution)
        #self.map[int((1+y_org)//self.resolution)][int(0+x_org//self.resolution)]=100

        max_len = np.max(self.ranges[np.isfinite(self.ranges)])
        self.get_logger().info(f"Max range length: {max_len}")
        for l,w in zip(self.ranges,self.angles):

            t=self.resolution
            while t < l and t < max_len:
                x_test = self.x + t*np.cos(w+self.w)
                y_test = self.y + t*np.sin(w+self.w)

                # Set the selected cell to be occupied (value 100)
                self.map[int((y_test+y_org)//self.resolution)][int((x_test+x_org)//self.resolution)] = 0
            
                t+=self.resolution*0.5

            x_test = self.x + t*np.cos(w+self.w)
            y_test = self.y + t*np.sin(w+self.w)

            # Set the selected cell to be occupied (value 100)
            if (l <= max_len):
                self.map[int((y_test+y_org)//self.resolution)][int((x_test+x_org)//self.resolution)] = 100



    def publish_map(self):
        # Populate random cells with obstacles (value 100)
        self.populate_obstacles()
        
        # Create an OccupancyGrid message
        occupancy_grid_msg = OccupancyGrid()

        # Fill in the header
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'map'  # The coordinate frame of the map
        
        # Fill in the map info
        occupancy_grid_msg.info.resolution = self.resolution  # resolution of the map
        occupancy_grid_msg.info.width = self.map_width
        occupancy_grid_msg.info.height = self.map_height
        occupancy_grid_msg.info.origin = Pose()
        occupancy_grid_msg.info.origin.position.x = -(self.map_width/2)*self.resolution  # Set the desired X coordinate
        occupancy_grid_msg.info.origin.position.y = -(self.map_height/2)*self.resolution# Set the desired Y coordinate
        occupancy_grid_msg.info.origin.position.z = 0.0  # Set the desired Z coordinate
        # occupancy_grid_msg.info.origin.orientation.x = 0.0  # No rotation
        # occupancy_grid_msg.info.origin.orientation.y = 0.0
        # occupancy_grid_msg.info.origin.orientation.z = 0.0
        # occupancy_grid_msg.info.origin.orientation.w = 1.0  # Identity quaternion (no rotation) # Set origin to (0, 0, 0) with no rotation
        flattened_data = [cell for row in self.map for cell in row]
        # Flatten the 2D matrix to 1D array
        flattened_data = [cell for row in self.map for cell in row]

        # Set the map data (Occupied cells = 100, Free cells = 0)
        occupancy_grid_msg.data = flattened_data

        # Publish the message
        self.publisher.publish(occupancy_grid_msg)
        self.get_logger().info('Map published on /map topic')

    def broadcast_static_transform(self):
        # Create the static transform from 'odom' frame to 'map' frame
        static_transform = TransformStamped()

        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'odom'  # Parent frame
        static_transform.child_frame_id = 'map'   # Child frame
        
        # Set translation (position) and rotation (orientation)
        static_transform.transform.translation.x = 0.0  # Map origin X
        static_transform.transform.translation.y = 0.0  # Map origin Y
        static_transform.transform.translation.z = 0.0  # Map origin Z
        static_transform.transform.rotation.x = 0.0    # No rotation
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0    # Identity quaternion (no rotation)

        # Send the transform
        self.tf_broadcaster.sendTransform(static_transform)
        #self.get_logger().info("Static transform broadcasted from 'odom' to 'map'")

def main(args=None):
    rclpy.init(args=args)
    
    map_publisher_node = MapPublisherNode()
    
    # Keep the node running
    rclpy.spin(map_publisher_node)
    
    # Shutdown after spinning
    map_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
