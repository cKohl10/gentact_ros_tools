#!/usr/bin/env python3
"""
PURPOSE:
    Aggregates obstacle points from multiple /obstacle_{link_name} topics
    and provides them for adding to PointArray objects at 10 Hz.
    
    Subscribes to: /obstacle_link2, /obstacle_link3, /obstacle_link4, /obstacle_link5, /obstacle_link6
    Publishes to: /aggregated_obstacle_poses (standard PoseStamped format)
    
    The published poses can be consumed by the ROS1 pose2pointarray.py script
    via the ROS1/ROS2 bridge to create PointArray messages.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header
import threading

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
        
        # Create publisher for aggregated obstacles as PoseStamped messages
        # These will be bridged to ROS1 and converted to PointArray by pose2pointarray.py
        self.aggregated_pose_pub = self.create_publisher(
            PoseStamped,
            '/aggregated_obstacle_poses',
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
        """Timer callback to aggregate all current obstacle points and publish at 10 Hz"""
        with self.obstacles_lock:
            # Collect all current obstacle points
            current_points = []
            current_time = self.get_clock().now()
            
            for link_name, obstacle_data in self.latest_obstacles.items():
                if obstacle_data is not None:
                    # Check if the data is recent (within last 0.5 seconds)
                    time_diff = current_time - obstacle_data['timestamp']
                    if time_diff.nanoseconds / 1e9 < 0.5:  # 500ms timeout
                        current_points.append({
                            'point': obstacle_data['point'],
                            'link_name': link_name,
                            'frame_id': obstacle_data['frame_id']
                        })
            
            # Publish each obstacle point as individual PoseStamped messages
            # These can be processed by pose2pointarray.py via ROS1/ROS2 bridge
            if current_points:
                self.get_logger().debug(f'Publishing {len(current_points)} aggregated obstacle points as individual poses')
                
                for obs in current_points:
                    # Create PoseStamped message for each obstacle point
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = obs['frame_id']
                    
                    # Set position from the point
                    pose_msg.pose.position = obs['point']
                    
                    # Set default orientation (no rotation)
                    pose_msg.pose.orientation.x = 0.0
                    pose_msg.pose.orientation.y = 0.0
                    pose_msg.pose.orientation.z = 0.0
                    pose_msg.pose.orientation.w = 1.0
                    
                    # Publish each pose individually
                    # The ROS1 pose2pointarray.py script can subscribe to this topic
                    # and convert individual poses to PointArray format
                    self.aggregated_pose_pub.publish(pose_msg)
                    
                    point = obs['point']
                    self.get_logger().debug(f'  Published {obs["link_name"]}: ({point.x:.3f}, {point.y:.3f}, {point.z:.3f}) in {obs["frame_id"]}')
            else:
                self.get_logger().debug('No obstacle points to publish')
            
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
