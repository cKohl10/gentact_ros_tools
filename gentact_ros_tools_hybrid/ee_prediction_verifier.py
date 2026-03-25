# This node reads training data from CSV and publishes it as a point cloud for coordinate frame verification
import rclpy
from rclpy.node import Node
import numpy as np
import csv
from sensor_msgs.msg import PointCloud2, PointField
import struct
import os

class TrainingDataPublisher(Node):
    def __init__(self):
        super().__init__('training_data_publisher')
        
        # Declare parameters
        self.declare_parameter('csv_path', '/home/carson/datasets/self-cap/calibration_tests/sphere_calibrations/link5_3/training_data.csv')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('frame_id', 'end_effector_tip')
        
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Load training data
        self.training_data = self._load_training_data(csv_path)
        
        # Create publisher
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/training_data_points', 1)
        
        # Create timer
        self.timer = self.create_timer(1.0/publish_rate, self.publish_callback)
        
        self.get_logger().info(f'Training Data Publisher Initialized')
        self.get_logger().info(f'Loaded {len(self.training_data)} data points from {csv_path}')
        self.get_logger().info(f'Publishing to frame: {self.frame_id}')
        self.get_logger().info(f'Publishing continuously at {publish_rate} Hz')
    
    def _load_training_data(self, csv_path):
        """Load training data from CSV file using built-in csv module."""
        try:
            if not os.path.exists(csv_path):
                self.get_logger().error(f'CSV file not found: {csv_path}')
                return np.array([])
            
            coords = []
            
            # Read the CSV file
            with open(csv_path, 'r') as csvfile:
                reader = csv.reader(csvfile)
                
                # Read header to find column indices
                header = next(reader)
                self.get_logger().info(f'CSV columns: {header}')
                
                # Find column indices for coordinates
                try:
                    x_idx = header.index('ee_x')
                    y_idx = header.index('ee_y')
                    z_idx = header.index('ee_z')
                except ValueError as e:
                    self.get_logger().error(f'Required columns not found in CSV: {e}')
                    return np.array([])
                
                # Read coordinate data
                for row_num, row in enumerate(reader):
                    try:
                        x = float(row[x_idx])
                        y = float(row[y_idx])
                        z = float(row[z_idx])
                        coords.append([x, y, z])
                    except (ValueError, IndexError) as e:
                        self.get_logger().warn(f'Error parsing row {row_num + 2}: {e}')
                        continue
            
            if len(coords) == 0:
                self.get_logger().error('No valid coordinate data found in CSV')
                return np.array([])
            
            coords = np.array(coords)
            self.get_logger().info(f'Loaded {len(coords)} coordinate points')
            
            # Log some statistics
            self.get_logger().info(f'Coordinate ranges:')
            self.get_logger().info(f'  X: [{coords[:, 0].min():.3f}, {coords[:, 0].max():.3f}]')
            self.get_logger().info(f'  Y: [{coords[:, 1].min():.3f}, {coords[:, 1].max():.3f}]')
            self.get_logger().info(f'  Z: [{coords[:, 2].min():.3f}, {coords[:, 2].max():.3f}]')
            
            return coords
                
        except Exception as e:
            self.get_logger().error(f'Error loading training data: {e}')
            return np.array([])
    
    def publish_callback(self):
        """Publish training data as point cloud continuously."""
        if len(self.training_data) == 0:
            return
        
        # Always publish all points continuously
        pointcloud_msg = self._create_pointcloud2_message(self.training_data)
        self.pointcloud_pub.publish(pointcloud_msg)
        self.get_logger().debug(f'Published {len(self.training_data)} points continuously')
    
    def _create_pointcloud2_message(self, points):
        """Create a PointCloud2 message from numpy array of points."""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Define the point fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Set the point cloud properties
        msg.point_step = 12  # 3 floats * 4 bytes each
        msg.row_step = 12 * len(points)  # point_step * number of points
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        
        # Pack the point data
        point_data = b''
        for point in points:
            point_data += struct.pack('fff', point[0], point[1], point[2])
        
        msg.data = point_data
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = TrainingDataPublisher()
    rclpy.spin(node)
    rclpy.shutdown()