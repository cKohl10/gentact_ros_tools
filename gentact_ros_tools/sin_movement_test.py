import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Float64MultiArray
from math import sin, pi

class SinMovementTest(Node):
    def __init__(self):
        super().__init__('sin_movement_test')

        self.declare_parameter('arm_id', 'fr3')
        self.arm_id = self.get_parameter('arm_id').get_parameter_value().string_value

        self.vel_pub = self.create_publisher(Float64MultiArray, f'{self.arm_id}_joint_velocity_controller/commands', 10)
        self.timer = self.create_timer(0.02, self.publish_velocities)
        self.get_logger().info('SinMovementTest node initialized')

    def publish_velocities(self):
        vel_msg = Float64MultiArray()
        vel_msg.data = [sin(self.get_clock().now().to_msg().sec * 2 * pi) * 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.vel_pub.publish(vel_msg)

    def joint_callback(self, msg):
        try:
            joint_state_msg = JointState()
            joint_state_msg.header = msg.header
            joint_state_msg.name = [f'{self.arm_id}_joint1', f'{self.arm_id}_joint2', f'{self.arm_id}_joint3', f'{self.arm_id}_joint4', f'{self.arm_id}_joint5', f'{self.arm_id}_joint6', f'{self.arm_id}_joint7']
            joint_state_msg.position = [msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6], msg.position[7], msg.position[8]]
            self.joint_pub.publish(joint_state_msg)
        except Exception as e:
            self.get_logger().error(f'Error in joint_callback: {e}')
            self.get_logger().error(f'msg: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = SinMovementTest()
    rclpy.spin(node)
    node.destroy_node()