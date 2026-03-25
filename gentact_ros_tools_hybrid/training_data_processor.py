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


class TrainingDataProcessor(Node):
    def __init__(self):
        super().__init__("training_data_processor")

        self.output_file = f'training_data.csv'
        self.declare_parameter('num_sensors', 6)
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        
        # Log the configuration
        self.get_logger().info(f"Training data processor configured for {self.num_sensors} sensors")
        
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
            # Create flat list of headers: timestamp, sensor_0_cc, sensor_1_cc, ..., ee_x, ee_y, ee_z
            headers = ['timestamp']
            headers.extend([f'sensor_{i}_cc' for i in range(self.num_sensors)])
            headers.extend(['ee_x', 'ee_y', 'ee_z'])
            writer.writerow(headers)
            self.get_logger().info(f"Created new {self.output_file} with headers for {self.num_sensors} sensors")

    def update_sensor_count(self, new_count: int):
        """Update the number of sensors and recreate the CSV file"""
        if new_count != self.num_sensors:
            self.get_logger().info(f"Updating sensor count from {self.num_sensors} to {new_count}")
            self.num_sensors = new_count
            self.setup_csv_file()

    def sensor_callback(self, msg: Int32MultiArray):
        # Validate that the message contains the expected number of sensors
        if len(msg.data) != self.num_sensors:
            self.get_logger().warning(
                f"Expected {self.num_sensors} sensors but received {len(msg.data)}. "
                f"Adjusting to use first {self.num_sensors} values."
            )
            # Use only the first num_sensors values
            sensor_data = list(msg.data[:self.num_sensors])
        else:
            sensor_data = list(msg.data)

        # Create flat list of data: timestamp, sensor_0_cc, sensor_1_cc, ..., ee_x, ee_y, ee_z
        # Use ROS clock time which respects --clock simulation
        ros_time = self.get_clock().now()
        data_log = [ros_time.nanoseconds / 1e9]

        # Add capacitance values for each sensor
        data_log.extend(sensor_data)

        # Add end effector position
        ee_pose = self._lookup_latest_transform('calibration_skin', 'end_effector_tip')
        if ee_pose is None:
            # Skip logging this cycle if TF not available yet
            return
        data_log.extend([
            ee_pose.transform.translation.x,
            ee_pose.transform.translation.y,
            ee_pose.transform.translation.z,
        ])

        self.write_to_csv(data_log)
    
    def write_to_csv(self, log):
        """Write data to CSV file"""
        try:
            with open(self.output_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(log)
        except Exception as e:
            self.get_logger().error(f"Error writing to CSV: {str(e)}")

    def _lookup_latest_transform(self, target_frame: str, source_frame: str):
        """Return the most recent transform available in the buffer or None.
        Uses a short timeout and avoids interpolation requests by asking for time=0.
        """
        try:
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


def main(args=None):
    rclpy.init(args=args)
    
    processor = TrainingDataProcessor()
    
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
