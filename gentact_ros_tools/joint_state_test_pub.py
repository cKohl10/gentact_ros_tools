import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateTestPub(Node):
    def __init__(self):
        super().__init__('joint_state_test_pub')
        self.get_logger().info('Joint State Test Pub node initialized')
        
        self.declare_parameter('arm_id', 'fr3')
        self.arm_id = self.get_parameter('arm_id').get_parameter_value().string_value
        
        self.joint_state_pub = self.create_publisher(JointState, f'joint_states_{self.arm_id}', 10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = [f'{self.arm_id}_joint1', f'{self.arm_id}_joint2', f'{self.arm_id}_joint3', f'{self.arm_id}_joint4', f'{self.arm_id}_joint5', f'{self.arm_id}_joint6', f'{self.arm_id}_joint7']

        self.timer = self.create_timer(5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        positions_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        positions_2 = [0.0, -0.785, 0.0, -2.356, 0.0, 1.6, 0.785]
        if self.counter % 2 == 0:
            self.joint_state_msg.position = positions_1
        else:
            self.joint_state_msg.position = positions_2
        self.joint_state_pub.publish(self.joint_state_msg)
        self.counter += 1
    

def main(args=None):
    rclpy.init(args=args)
    joint_state_test_pub = JointStateTestPub()
    rclpy.spin(joint_state_test_pub)