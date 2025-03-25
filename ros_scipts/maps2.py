
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, TransformStamped, Point
from std_msgs.msg import Header
import random
import tf2_ros

class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher_node')


        # Define the map size (10x10 matrix as an example)
        self.resolution = 0.05  # resolution of 0.5 meters per grid cell
        self.map_width = int(20*(1//self.resolution))
        self.map_height = int(20*(1//self.resolution))

        # Initialize the map with free cells (value 0)
        self.map = [[0 for _ in range(self.map_width)] for _ in range(self.map_height)]
        
        # Populate random cells with obstacles (value 100)
        self.populate_obstacles()

        # Create a publisher for the /map topic
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Timer to publish map at 0.5 Hz
        self.timer = self.create_timer(2.0, self.publish_map)

        # Create a static transform broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Broadcast a static transform from 'odom' frame to 'map' frame
        self.broadcast_static_transform()

    def populate_obstacles(self):
        # Define the number of obstacles to add
        num_obstacles = 20
        
        for _ in range(num_obstacles):
            # Choose a random position on the map
            x = random.randint(0, self.map_width - 1)
            y = random.randint(0, self.map_height - 1)
            
            # Set the selected cell to be occupied (value 100)
            self.map[y][x] = 100

    def publish_map(self):
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
        self.get_logger().info("Static transform broadcasted from 'odom' to 'map'")

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
