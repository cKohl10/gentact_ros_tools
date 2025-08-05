import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateTestPub(Node):
    def __init__(self):
        super().__init__('joint_state_test_pub')
        self.get_logger().info('Joint State Test Pub node initialized')
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

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