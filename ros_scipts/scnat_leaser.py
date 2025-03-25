import rclpy
from rclpy.node import Node
from sensor_msgs.msg import laserscan
import numpy as np
from geometry_msgs.msg import transformstamped
#import tf_transformations
import tf2_ros
#import open3d as o3d

class lidarprocessor(Node):

    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            laserscan,
            '/scan',
            self.listener_callback,
            10)

        #self.subscription  # prevent unused variable warning

        # transform broadcaster
        self.tf_broadcaster = tf2_ros.transformbroadcaster(self)
        
        # initial robot pose (x, y, yaw)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.prev_scan = none  # store previous scan for comparison
        self.prev_range = none

    def listener_callback(self, msg):
        #current_scan = msg.ranges #floeat32[]
        #angel_min= msg.angle_min #floeat32
        #angel_max = msg.angle_max #floeat32
        #angel_inclement= msg.angle_increment#floeat32
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        #can be a problem if i need to compare whit priv and not same size
        if self.prev_range is not none:
            valid_indices = ~np.isnan(ranges) & np.isfinite(ranges) & (ranges > 0) & np.isfinite(self.prev_range)
        else:
            valid_indices = ~np.isnan(ranges) & np.isfinite(ranges) & (ranges > 0)

        self.prev_range = np.array(msg.ranges)
        x_points = ranges * np.cos(angles)
        y_points = ranges * np.sin(angles)

        x_points = x_points[valid_indices]
        y_points = y_points[valid_indices]
        
        scan_points = np.vstack((x_points, y_points)).t  # nx2 matrix

        if self.prev_scan is not none:
            # estimate transformation (using simple mean shift or icp)
            dx, dy, dyaw = self.estimate_movement(self.prev_scan, scan_points)
            
            # update robot pose
            self.x += dx
            self.y += dy
            self.yaw += dyaw

        self.prev_scan = scan_points  # store current scan for next iteration
        
        self.get_logger().info(f' {self.x}, {self.y}, {self.yaw}')
        # publish transform
        #self.publish_tf()
        
    def estimate_movement(self, prev_scan, curr_scan):
        """ estimate movement between scans. uses a simple centroid shift method. """
        prev_mean = np.mean(prev_scan, axis=0)
        curr_mean = np.mean(curr_scan, axis=0)

        dx = curr_mean[0] - prev_mean[0]
        dy = curr_mean[1] - prev_mean[1]

        # Implementing a simple ICP (Iterative Closest Point) algorithm
        def icp(a, b, max_iterations=20, tolerance=1e-6):
            src = np.array([a.T], copy=True).astype(np.float32)
            dst = np.array([b.T], copy=True).astype(np.float32)

            prev_error = 0

            for i in range(max_iterations):
                # Find the nearest neighbors between the current source and destination points
                distances = np.linalg.norm(src[:, :, np.newaxis] - dst[:, np.newaxis, :], axis=1)
                indices = np.argmin(distances, axis=2)

                # Compute the transformation between the current source and nearest destination points
                T, _, _ = np.linalg.svd(np.dot(dst[0, :, indices[0]].T, src[0].T))

                # Update the current source
                src = np.dot(T, src[0].T).T[np.newaxis, :, :]

                # Check for convergence
                mean_error = np.mean(distances)
                if np.abs(prev_error - mean_error) < tolerance:
                    break
                prev_error = mean_error

            # Calculate the final transformation
            T, _, _ = np.linalg.svd(np.dot(dst[0, :, indices[0]].T, src[0].T))
            R = T[:2, :2]
            t = T[:2, 2]

            return R, t

        R, t = icp(prev_scan, curr_scan)
        dx = t[0]
        dy = t[1]
        dyaw = np.arctan2(R[1, 0], R[0, 0])

        return dx, dy, dyaw

          

    def publish_tf(self):
        """ publish the robot pose as a transform (tf) """
        t = transformstamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "my_odom"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        # t.transform.rotation.x = q[0]
        # t.transform.rotation.y = q[1]
        # t.transform.rotation.z = q[2]
        # t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendtransform(t)

def main(args=none):
    rclpy.init(args=args)
    lidar_processor = lidarprocessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()