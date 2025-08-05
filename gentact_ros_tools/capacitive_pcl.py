import math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray, Float64, Float64MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
from typing import List
import os
from ament_index_python.packages import get_package_share_directory
import json
from tf2_ros import TransformListener, Buffer

class CapacitivePCL(Node):
    def __init__(self):
        super().__init__("capacitive_pcl")

        default_config_path = os.path.join(get_package_share_directory('gentact_ros_tools'), 'config', 'link5_sphere_1.json')
        
        # Declare parameters
        self.declare_parameter('num_sensors', 6)
        self.declare_parameter('max_distance', 0.1)  # Maximum distance threshold in meters (10 cm)
        self.declare_parameter('sensor_config', default_config_path)
        self.declare_parameter('frame_id', 'link5')

        # Get parameters
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        self.sensor_config = self.get_parameter('sensor_config').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.base_points_per_sensor = 1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # NOTE: To prevent overflow, we prematurely apply at 10^13 magnatude shift due to approximate the permittivity of air
        self.clock_freq = 160.0  # 160 MHz clock frequency for Qwiic board
        self.R = [10.0] * self.num_sensors  # Resistance values for each sensor
        self.alpha = 0.0005  # Initial tuning parameter

        self.envelopes = [1.0] * self.num_sensors

        # Load sensor configuration
        with open(self.sensor_config, 'r') as f:
            self.sensor_config = json.load(f)
            self.sensor_remapping = self.sensor_config['remappings']

        self.tuning_sub = self.create_subscription(
            Float64,
            '/sensor_tuning',
            self.tuning_callback,
            1
        )
        self.tuning_sub  # prevent unused variable warning

        # self.tracking_sub = self.create_subscription(
        #     Float64MultiArray,
        #     '/sensor_tracking',
        #     self.sensor_callback,
        #     1
        # )

        self.tracking_sub = self.create_subscription(
            Int32MultiArray,
            '/sensor_raw',
            self.sensor_callback,
            1
        )
        self.tracking_sub  # prevent unused variable warning

        # Create publishers for each sensor's pointcloud
        self.pcl_publishers = []
        for i in range(self.num_sensors):
            publisher = self.create_publisher(
                PointCloud2,
                f'/sensor_{i}_pcl',
                10
            )
            self.pcl_publishers.append(publisher)

        self.dist_pub = self.create_publisher(
            Float64MultiArray,
            '/sensor_dist',
            1
        )

        self.obstacle_pub = self.create_publisher(
            PoseStamped,
            '/obstacle',
            1
        )

        self.baseline_sub = self.create_subscription(
            Int32MultiArray,
            '/sensor_baseline',
            self.baseline_callback,
            1
        )
        self.baseline_sub  # prevent unused variable warning
        
        # Initialize base pointcloud pattern (small grid)
        self.base_points = self.generate_base_points()
        
        self.get_logger().info(f"Capacitive PCL node initialized for {self.num_sensors} sensors")

    def tuning_callback(self, msg: Float64):
        self.alpha = float(msg.data)
        self.get_logger().info(f"Received tuning alpha: {self.alpha}")
    
    def baseline_callback(self, msg: Int32MultiArray):
        self.baseline = msg.data
        self.get_logger().info(f"Received baseline: {self.baseline}")
    
    def generate_base_points(self):
        """Generate a base pattern of points for each sensor"""
        # Create a small grid of points in XY plane
        grid_size = int(np.sqrt(self.base_points_per_sensor))
        if grid_size * grid_size < self.base_points_per_sensor:
            grid_size += 1

        x = [0]
        y = [0]  # 1cm spread in Y

        points = []
        count = 0
        for xi in x:
            for yi in y:
                if count < self.base_points_per_sensor:
                    points.append([xi, yi, 0.0])  # Z will be modified by distance
                    count += 1
        
        return np.array(points)
    
    def create_pointcloud_msg(self, point, sensor_id):
        """Create a PointCloud2 message from numpy array of points"""
        # Create header
        msg = PointCloud2()
        # Use a timestamp slightly in the past to ensure TF transforms are available
        current_time = self.get_clock().now()
        # Subtract 0.1 seconds to ensure transforms are available
        past_time = current_time - Duration(seconds=0.1)
        msg.header.stamp = past_time.to_msg()
        msg.header.frame_id = f"{self.frame_id}_sensor_{self.sensor_remapping[sensor_id]}"
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Set up the cloud data
        msg.height = 1
        msg.width = 1
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_bigendian = False
        msg.is_dense = True
        
        # Pack the data
        buffer = []
        buffer.append(struct.pack('fff', point[0], point[1], point[2]))
        
        msg.data = b''.join(buffer)
        
        return msg
    
    def sensor_callback(self, msg):
        """Callback function for sensor data"""
        try:
            # Get sensor values
            sensor_values = msg.data
            
            # Ensure we have the expected number of sensors
            num_available = min(len(sensor_values), self.num_sensors)
            dist_values = []
            all_sensor_points = []
            raw_dist_values = []
            
            for i in range(num_available):
                # Get distance value and convert to meters
                # distance = sensor_values[i] * self.distance_scale
                capacitance = -(np.abs(sensor_values[i] - self.baseline[i])) / (self.clock_freq * self.R[i] * 30.0 * math.log(0.5))
                distance = self.alpha / capacitance
                raw_dist_values.append(distance)

                # Create pointcloud for this sensor
                # Copy base points and offset them by the distance in Z
                sensor_points = np.array([0.0, 0.0, 0.0])
                
                # if distance <= self.max_distance and distance >= 0.0:
                #     # Within range: use actual distance
                #     sensor_points[2] = distance
                #     self.get_logger().debug(f"Sensor {i} within range: {distance:.6f}m")
                # else:
                #     # Out of range: publish at -0.2 * max_distance
                #     sensor_points[2] = -0.2 * self.max_distance
                #     self.get_logger().debug(f"Sensor {i} out of range ({distance:.6f}m), publishing at {-0.2 * self.max_distance:.6f}m")
                #     distance = self.max_distance + 0.01
                
                #Get Lowest distance sensor
                # Create and publish pointcloud message
                # pcl_msg = self.create_pointcloud_msg(sensor_points, i)
                # self.pcl_publishers[i].publish(pcl_msg)
                sensor_points[2] = distance
                all_sensor_points.append(sensor_points)
                dist_values.append(distance)
            # dist_msg = Float64MultiArray(data=dist_values)
            # self.dist_pub.publish(dist_msg)
            
            self.get_logger().debug(f"Published pointclouds for {num_available} sensors")

            # Get lowest distance sensor
            min_distance_sensor = np.argmin(raw_dist_values)
            min_distance = raw_dist_values[min_distance_sensor]
            # self.get_logger().info(f"Lowest distance sensor: {min_distance_sensor}")

            if min_distance < self.max_distance:
                msg = self.get_obstacle_pose(min_distance_sensor, all_sensor_points[min_distance_sensor])
                self.obstacle_pub.publish(msg)
            else:
                zero_pose = PoseStamped()
                zero_pose.header.stamp = self.get_clock().now().to_msg()
                zero_pose.header.frame_id = 'map'
                zero_pose.pose.position.x = 100.0
                zero_pose.pose.position.y = 0.0
                zero_pose.pose.position.z = 0.0
                self.obstacle_pub.publish(zero_pose)

        except Exception as e:
            self.get_logger().error(f"Error in sensor_callback: {e}")

    def get_obstacle_pose(self, sensor_id, sensor_point):
        """Get the pose of the obstacle"""
        map_transform = self.tf_buffer.lookup_transform(
            "map",
            f"{self.frame_id}_sensor_{self.sensor_remapping[sensor_id]}",
            rclpy.time.Time()
        )

        transformed_point = self._transform_point(sensor_point, map_transform)

        """Create an obstacle message from a single 3D point."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Always publish in 'map' frame

        # Use the already transformed point
        msg.pose.position.x = float(transformed_point[0])
        msg.pose.position.y = float(transformed_point[1])
        msg.pose.position.z = float(transformed_point[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        return msg

    def _transform_point(self, point, transform):
            """Transform a point from sensor frame to map frame using rotation and translation."""
            # Extract quaternion and translation from transform
            q = transform.transform.rotation
            t = transform.transform.translation
            
            # Convert quaternion to rotation matrix manually
            # Quaternion components: [w, x, y, z]
            w, x, y, z = q.w, q.x, q.y, q.z
            
            # Normalize quaternion
            norm = np.sqrt(w*w + x*x + y*y + z*z)
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
            
            # Rotation matrix from quaternion
            rotation_matrix = np.array([
                [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y],
                [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x],
                [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
            ])
            
            # Apply rotation first, then translation
            rotated_point = rotation_matrix @ point
            transformed_point = rotated_point + np.array([t.x, t.y, t.z])
            
            return transformed_point

    def update_envelopes(self, values):
        for i in range(self.num_sensors):
            if values[i] > self.envelopes[i]:
                self.envelopes[i] = values[i]
            else:
                self.envelopes[i] = self.envelopes[i] * 0.99

def main(args=None):
    rclpy.init(args=args)
    
    capacitive_pcl = CapacitivePCL()
    
    try:
        rclpy.spin(capacitive_pcl)
    except KeyboardInterrupt:
        pass
    finally:
        capacitive_pcl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
