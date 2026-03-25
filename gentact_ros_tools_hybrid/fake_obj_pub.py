"""
PURPOSE:
    creates an obstacle that moves in a circle 
    intended to be used with the avoidance controler
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

class FakeObjPub(Node):
    def __init__(self):
        super().__init__('fake_obj_pub')

        self.point_pub = self.create_publisher(
            PoseStamped,
            '/obstacle',
            10
        )

        # Circular motion parameters
        self.radius = 0.4  # meters
        self.center_x = 0.0
        self.center_y = 0.0
        self.center_z = 0.4
        self.angular_speed = 0.5  # radians per second
        self.current_angle = 0.0
        
        # Create timer for periodic updates (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('FakeObjPub node initialized - publishing circular motion')

        self.point = Point()
        self.pose = PoseStamped()
        self.pose.pose.position = self.point

    def timer_callback(self):
        # Calculate new position on circle
        self.point.x = self.center_x + self.radius * math.cos(self.current_angle)
        self.point.y = self.center_y + self.radius * math.sin(self.current_angle)
        self.point.z = self.center_z
        
        # Update angle for next iteration
        self.current_angle += self.angular_speed * 0.1  # 0.1 seconds per update
        
        # Keep angle in reasonable range
        if self.current_angle > 2 * math.pi:
            self.current_angle -= 2 * math.pi
            
        # Publish the updated point
        self.point_pub.publish(self.pose)

def main():
    rclpy.init()
    node = FakeObjPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()