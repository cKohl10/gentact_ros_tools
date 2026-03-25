#!/usr/bin/env python3
"""
PURPOSE:
    Aggregates obstacle points from multiple /obstacle_{link_name} topics
    and publishes them as a PointCloud2 message at 10 Hz.
    
    Subscribes to: /obstacle_link2, /obstacle_link3, /obstacle_link4, /obstacle_link5, /obstacle_link6
    Publishes to: /obstacle_pointcloud (sensor_msgs/PointCloud2)
    
    PointCloud2 is used for efficient storage and transmission of 3D point arrays.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import threading
import struct

class ObstacleAggregatorNode(Node):
    def __init__(self):
        super().__init__('obstacle_aggregator')
        
        # Define the active link names based on collision.yaml config
        self.active_links = ['link2', 'link3', 'link4', 'link5', 'link6']
        # self.active_links = range(8)
        
        # Storage for latest obstacle points from each link
        self.latest_obstacles = {}
        self.obstacles_lock = threading.Lock()
        
        # Initialize storage for each link
        for link_name in self.active_links:
            self.latest_obstacles[link_name] = None
        
        # Create subscribers for each obstacle topic
        self.obstacle_subscribers = {}
        for link_name in self.active_links:
            topic_name = f'/obstacle_{link_name}'
            self.obstacle_subscribers[link_name] = self.create_subscription(
                PoseStamped,
                topic_name,
                lambda msg, link=link_name: self.obstacle_callback(msg, link),
                10
            )
            self.get_logger().info(f'Subscribed to {topic_name}')
        
        # Create publisher for aggregated obstacles as PointCloud2
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/obstacle_pointcloud',
            10
        )
        
        # Create timer that runs at 10 Hz to aggregate and publish
        self.timer = self.create_timer(0.1, self.aggregate_and_publish)  # 10 Hz
        
        self.get_logger().info('Obstacle Aggregator Node initialized')
        self.get_logger().info(f'Monitoring obstacles from: {self.active_links}')
    
    def obstacle_callback(self, msg: PoseStamped, link_name: str):
        """Callback to store the latest obstacle point from a specific link"""
        with self.obstacles_lock:
            # Extract the point from the PoseStamped message
            point = Point()
            point.x = msg.pose.position.x
            point.y = msg.pose.position.y
            point.z = msg.pose.position.z
            
            # Store the latest point for this link
            self.latest_obstacles[link_name] = {
                'point': point,
                'timestamp': self.get_clock().now(),
                'frame_id': msg.header.frame_id
            }
            
            self.get_logger().debug(f'Received obstacle from {link_name}: ({point.x:.3f}, {point.y:.3f}, {point.z:.3f})')
    
    def aggregate_and_publish(self):
        """Timer callback to aggregate all current obstacle points and publish as PointCloud2 at 10 Hz"""
        with self.obstacles_lock:
            # Collect all current obstacle points
            current_points = []
            current_time = self.get_clock().now()
            frame_id = 'world'  # Default frame
            
            for link_name, obstacle_data in self.latest_obstacles.items():
                if obstacle_data is not None:
                    # Check if the data is recent (within last 0.5 seconds)
                    time_diff = current_time - obstacle_data['timestamp']
                    if time_diff.nanoseconds / 1e9 < 0.5:  # 500ms timeout
                        current_points.append(obstacle_data['point'])
                        frame_id = obstacle_data['frame_id']  # Use frame from obstacles
            
            # Publish all points as a single PointCloud2 message
            if current_points:
                pointcloud_msg = self.create_pointcloud2(current_points, frame_id)
                self.pointcloud_pub.publish(pointcloud_msg)
                
                self.get_logger().debug(f'Published PointCloud2 with {len(current_points)} obstacle points in {frame_id}')
            else:
                self.get_logger().debug('No obstacle points to publish')
    
    def create_pointcloud2(self, points, frame_id='world'):
        """Create a PointCloud2 message from a list of Point objects"""
        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        # Define the fields (x, y, z as float32)
        msg.height = 1  # Unordered point cloud
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes each
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack point data as binary
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', point.x, point.y, point.z))
        
        msg.data = b''.join(buffer)
        
        return msg
            
    def get_current_obstacle_points(self):
        """Public method to get current obstacle points for external use"""
        with self.obstacles_lock:
            points = []
            current_time = self.get_clock().now()
            
            for link_name, obstacle_data in self.latest_obstacles.items():
                if obstacle_data is not None:
                    time_diff = current_time - obstacle_data['timestamp']
                    if time_diff.nanoseconds / 1e9 < 0.5:  # 500ms timeout
                        points.append(obstacle_data['point'])
            
            return points

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAggregatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
