
import rclpy
from rclpy.node import Node
import random

class MapNode(Node):
    def __init__(self):
        super().__init__('map_node')

        # Define the map size (10x10 matrix as an example)
        self.map_width = 10
        self.map_height = 10
        
        # Initialize the map with free cells (value 0)
        self.map = [[0 for _ in range(self.map_width)] for _ in range(self.map_height)]
        
        # Populate random cells with obstacles (value 100)
        self.populate_obstacles()

        # For visualization, let's print the map
        self.print_map()

    def populate_obstacles(self):
        # Define the number of obstacles to add
        num_obstacles = 20
        
        for _ in range(num_obstacles):
            # Choose a random position on the map
            x = random.randint(0, self.map_width - 1)
            y = random.randint(0, self.map_height - 1)
            
            # Set the selected cell to be occupied (value 100)
            self.map[y][x] = 100

    def print_map(self):
        for row in self.map:
            self.get_logger().info(' '.join(str(cell) for cell in row))

def main(args=None):
    rclpy.init(args=args)
    
    map_node = MapNode()
    
    # Keep the node running
    rclpy.spin(map_node)
    
    # Shutdown after spinning
    map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
