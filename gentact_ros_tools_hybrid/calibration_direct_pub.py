from math import sin, cos, pi
import numpy as np
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped, Buffer, TransformListener

class CalibrationDirectPub(Node):

    def __init__(self):
        super().__init__('calibration_direct_pub')

        # Declare parameter with default value
        self.declare_parameter('robot_namespace', 'calibration')
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # Create broadcaster
        qos_profile = QoSProfile(depth=10)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.target_pose_pub = self.create_publisher(TransformStamped, '/target_pose', 10)

        # Initialize sensor counter
        self.sensor_num = 0
        self.sensor_num_max = 7
        
        # Create timer for periodic execution
        self.timer = self.create_timer(5.0, self.sensor_callback)

    def sensor_callback(self):
        # Get the current sensor transform
        sensor_frame = self.robot_namespace + '_sensor_' + str(self.sensor_num)
        self.get_logger().info(f"Trying to get transform for sensor: {sensor_frame}")

        # Create transform with smooth lowering motion
        iterations = 50  # More iterations for smoother motion
        height = np.linspace(0.1, 0.01, iterations)
        for i in range(iterations):
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = sensor_frame
            transform.child_frame_id = 'target_pose'
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = height[i]
            transform.transform.rotation = euler_to_quaternion(0.0, 3.14159, 4.0)
            self.broadcaster.sendTransform(transform)
            self.target_pose_pub.publish(transform)
            time.sleep(0.05)  # 50ms delay for smooth motion
        self.get_logger().info(f"Published transform for sensor {self.sensor_num}")

        # Increment sensor number
        self.sensor_num = (self.sensor_num + 1) % self.sensor_num_max

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    rclpy.init()
    node = CalibrationDirectPub()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()