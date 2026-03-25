import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Panda2Fr3(Node):
    def __init__(self):
        super().__init__('panda2fr3')

        self.declare_parameter('arm_id', 'fr3')
        self.arm_id = self.get_parameter('arm_id').get_parameter_value().string_value

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            f'/joint_states_{self.arm_id}',
            10
        )

        self.get_logger().info('Panda2Fr3 node initialized')

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
    node = Panda2Fr3()
    rclpy.spin(node)
    node.destroy_node()