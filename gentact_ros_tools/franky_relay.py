#! /usr/bin/env python3

import sys
import os

# Try to find and use the virtual environment
def setup_venv():
    venv_path = "/home/caleb/ros2_ws/.venv"
    if os.path.exists(venv_path):
        # Add virtual environment's site-packages to Python path
        site_packages = os.path.join(venv_path, "lib", "python3.8", "site-packages")
        if os.path.exists(site_packages) and site_packages not in sys.path:
            sys.path.insert(0, site_packages)
        
        # Set environment variables
        os.environ['VIRTUAL_ENV'] = venv_path
        if 'PATH' in os.environ:
            os.environ['PATH'] = os.path.join(venv_path, 'bin') + ':' + os.environ['PATH']
        else:
            os.environ['PATH'] = os.path.join(venv_path, 'bin')

setup_venv()

try:
    from franky import *
except ImportError as e:
    print(f"Error importing franky: {e}")
    print(f"Python executable: {sys.executable}")
    print(f"Python path: {sys.path}")
    print(f"VIRTUAL_ENV: {os.environ.get('VIRTUAL_ENV', 'Not set')}")
    sys.exit(1)

import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
import numpy as np

class FrankyRelay(Node):
    def __init__(self):
        super().__init__('franky_relay')
        self.get_logger().info('Franky Relay node initialized')
        self.joint_sub = self.create_subscription(JointState, 'joint_states_fr3', self.joint_state_callback, 10)
        self.joint_states = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])


        self.robot = Robot("192.168.0.198")  # Replace this with your robot's IP
        self.robot.relative_dynamics_factor = 0.1
        self.robot.recover_from_errors()

        print(f'Moving to home position: {self.joint_states}')
        home_motion = JointMotion(self.joint_states)
        self.robot.move(home_motion)

    def joint_state_callback(self, msg):
        try:
            self.joint_states = np.array(msg.position[:7])
            self.get_logger().info(f'Received joint states: {self.joint_states}')
            current_joints = np.array(self.robot.current_joint_state.position[:7])
            print(f'Current joints: {current_joints}')
            print(f'Desired joints: {self.joint_states}')
            scaling = 1.0
            diff = (self.joint_states - current_joints) * scaling
            print(f'Diff: {diff=}')
            motion = JointVelocityMotion(
                diff, duration=Duration(1000),
            )
            self.robot.move(motion, asynchronous=True)
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {e}')
            self.robot.recover_from_errors()


def main(args=None):
    rclpy.init(args=args)
    franky_relay = FrankyRelay()
    rclpy.spin(franky_relay)
    franky_relay.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()