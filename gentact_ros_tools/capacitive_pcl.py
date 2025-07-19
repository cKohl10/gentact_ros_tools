import math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Int32MultiArray, Float64
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
from typing import List


class CapacitivePCL(Node):
    def __init__(self):
        super().__init__("capacitive_pcl")
        
        # Declare parameters
        self.declare_parameter('num_sensors', 6)
        self.declare_parameter('max_distance', 0.1)  # Maximum distance threshold in meters (10 cm)
        
        # Get parameters
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        self.base_points_per_sensor = 1

        # NOTE: To prevent overflow, we prematurely apply at 10^13 magnatude shift due to approximate the permittivity of air
        self.clock_freq = 160.0  # 160 MHz clock frequency for Qwiic board
        self.R = [10.0] * self.num_sensors  # Resistance values for each sensor
        self.alpha = 0.0005  # Initial tuning parameter

        # Dict incase nodes are in wrong place
        self.sensor_dict = {
            5: '4',
            3: '2',
            4: '5',
            0: '1',
            1: '6',
            2: '0',
        }


        
        # Subscribe to sensor data
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/sensor_raw',
            self.sensor_callback,
            1
        )

        self.tuning_sub = self.create_subscription(
            Float64,
            '/sensor_tuning',
            self.tuning_callback,
            1
        )
        self.tuning_sub  # prevent unused variable warning
        # Create publishers for each sensor's pointcloud
        self.pcl_publishers = []
        for i in range(self.num_sensors):
            publisher = self.create_publisher(
                PointCloud2,
                f'/sensor_{i}_pcl',
                10
            )
            self.pcl_publishers.append(publisher)
        
        # Initialize base pointcloud pattern (small grid)
        self.base_points = self.generate_base_points()
        
        self.get_logger().info(f"Capacitive PCL node initialized for {self.num_sensors} sensors")

    def tuning_callback(self, msg: Float64):
        self.alpha = float(msg.data)
        self.get_logger().info(f"Received tuning alpha: {self.alpha}")
    
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
    
    def create_pointcloud_msg(self, points, sensor_id):
        """Create a PointCloud2 message from numpy array of points"""
        # Create header
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"fr3_link6_sensor_{self.sensor_dict[sensor_id]}"
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Set up the cloud data
        msg.height = 1
        msg.width = len(points)
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_bigendian = False
        msg.is_dense = True
        
        # Pack the data
        buffer = []
        for point in points:
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
            
            for i in range(num_available):
                # Get distance value and convert to meters
                # distance = sensor_values[i] * self.distance_scale
                capacitance = -sensor_values[i] / (self.clock_freq * self.R[i] * 30 * math.log(0.5))
                distance = self.alpha / capacitance

                # Create pointcloud for this sensor
                # Copy base points and offset them by the distance in Z
                sensor_points = self.base_points.copy()
                
                if distance <= self.max_distance:
                    # Within range: use actual distance
                    sensor_points[:, 2] = distance
                    self.get_logger().debug(f"Sensor {i} within range: {distance:.6f}m")
                else:
                    # Out of range: publish at -0.2 * max_distance
                    sensor_points[:, 2] = -0.2 * self.max_distance
                    self.get_logger().debug(f"Sensor {i} out of range ({distance:.6f}m), publishing at {-0.2 * self.max_distance:.6f}m")
                
                # Create and publish pointcloud message
                pcl_msg = self.create_pointcloud_msg(sensor_points, i)
                self.pcl_publishers[i].publish(pcl_msg)
            
            self.get_logger().debug(f"Published pointclouds for {num_available} sensors")
            
        except Exception as e:
            self.get_logger().error(f"Error in sensor_callback: {e}")


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
