# This node processes the self capacitance data and stores it in a csv file.

import math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Int32MultiArray, Float64, Float64MultiArray
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
from typing import List
import tf2_ros
from geometry_msgs.msg import TransformStamped
import csv
import os
from rclpy.duration import Duration


class Processor(Node):
    def __init__(self):
        super().__init__("processor")

        self.output_file = f'data.csv'
        self.declare_parameter('num_sensors', 6)
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        
        # TF lookup timeout and helper settings
        self.tf_timeout = Duration(seconds=0.2)
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.sensor_sub = self.create_subscription(
            Int32MultiArray,
            '/sensor_raw',
            self.sensor_callback,
            1
        )
        
        # CSV file setup
        self.setup_csv_file()
        
        self.get_logger().info("Processor node initialized with TF2 distance tracking")
    
    def setup_csv_file(self):
        """Initialize the CSV file with headers"""
        # Delete existing file if it exists
        if os.path.exists(self.output_file):
            os.remove(self.output_file)
            self.get_logger().info(f"Deleted existing {self.output_file}")
        
        # Create new file with headers
        with open(self.output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Create flat list of headers: timestamp, sensor_0_distance, sensor_1_distance, ..., sensor_0_cc, sensor_1_cc, ...
            headers = ['timestamp']
            headers.extend([f'sensor_{i}_distance' for i in range(self.num_sensors)])
            headers.extend([f'sensor_{i}_cc' for i in range(self.num_sensors)])
            writer.writerow(headers)
            self.get_logger().info(f"Created new {self.output_file} with headers")

    def sensor_callback(self, msg: Int32MultiArray):
        # Create flat list of data: timestamp, sensor_0_distance, sensor_1_distance, ..., sensor_0_cc, sensor_1_cc, ...
        # Use ROS clock time which respects --clock simulation
        ros_time = self.get_clock().now()
        data_log = [ros_time.nanoseconds / 1e9]
        
        # Add distance values for each sensor
        for i in range(self.num_sensors):
            distance = self.calculate_distance(f"calibration_sensor_{i}", "end_effector_tip")
            data_log.append(distance)

        # if any distance is -1, return
        if any(distance == -1 for distance in data_log[1:]):
            return

        # Add capacitance values for each sensor (flatten msg.data)
        data_log.extend(list(msg.data))
        self.write_to_csv(data_log)
    
    def calculate_distance(self, sensor_tf, ee_tf):
        """Calculate the cartesian distance between end_effector_tip and calibration_sensor_1"""
        
        try:
            # Get the latest available transform from sensor_tf to ee_tf
            transform = self._lookup_latest_transform(ee_tf, sensor_tf)
            if transform is None:
                return -1.0
            
            # Extract translation components
            translation = transform.transform.translation
            x, y, z = translation.x, translation.y, translation.z
            
            # Calculate cartesian distance
            distance = math.sqrt(x**2 + y**2 + z**2)
            return distance
        except Exception as e:
            self.get_logger().warning(f"Error calculating distance: {str(e)}")
            return -1.0
    
    def _lookup_latest_transform(self, target_frame: str, source_frame: str):
        """Return the most recent transform available in the buffer or None.
        Uses a short timeout and avoids interpolation requests by asking for time=0.
        """
        try:
            # Ask for the latest available transform (time=0) and wait briefly
            if self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), self.tf_timeout):
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    rclpy.time.Time(),
                    self.tf_timeout,
                )
            else:
                self.get_logger().warning(
                    f"Transform not available yet: {source_frame} -> {target_frame}"
                )
                return None
        except Exception as e:
            self.get_logger().warning(
                f"TF lookup failed for {source_frame} -> {target_frame}: {str(e)}"
            )
            return None
    
    def write_to_csv(self, log):
        """Write distance data to CSV file"""
        try:
            with open(self.output_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(log)
        except Exception as e:
            self.get_logger().error(f"Error writing to CSV: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    processor = Processor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
