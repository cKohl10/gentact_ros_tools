#!/tmp/franky_repo/venv-franky/bin/python3

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
from collections import deque

class FrankyRelay(Node):
    def __init__(self):
        super().__init__('franky_relay')
        self.get_logger().info('Franky Relay node initialized')
        
        # Declare parameters for controller gains
        self.declare_parameter('velocity_scaling', 3.0)
        self.declare_parameter('dynamics_factor', 0.3)
        self.declare_parameter('motion_duration_ms', 100)  # Increased for smoother motion
        self.declare_parameter('update_rate_hz', 20.0)      # Much slower for stability
        
        # Get parameter values
        self.velocity_scaling = self.get_parameter('velocity_scaling').get_parameter_value().double_value
        self.dynamics_factor = self.get_parameter('dynamics_factor').get_parameter_value().double_value
        self.motion_duration_ms = self.get_parameter('motion_duration_ms').get_parameter_value().integer_value
        self.update_rate_hz = self.get_parameter('update_rate_hz').get_parameter_value().double_value
        
        self.get_logger().info(f'Controller gains: velocity_scaling={self.velocity_scaling}, dynamics_factor={self.dynamics_factor}, duration={self.motion_duration_ms}ms, rate={self.update_rate_hz}Hz')
        
        # Store latest joint state data
        self.latest_joint_states = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.joint_states_updated = False
        
        # Velocity smoothing buffers
        self.velocity_history = deque(maxlen=5)  # Keep last 5 velocity commands
        self.last_velocity_command = np.zeros(7)
        self.max_velocity_change = 0.05  # Very conservative change limit
        self.command_count = 0
        
        # Subscribe to joint states topic
        self.joint_sub = self.create_subscription(JointState, 'joint_states_fr3', self.joint_state_callback, 1)
        
        # Create timer with configurable update rate
        self.timer = self.create_timer(1.0/self.update_rate_hz, self.timer_callback)

        self.robot = Robot("192.168.0.198")  # Replace this with your robot's IP
        self.robot.relative_dynamics_factor = self.dynamics_factor
        self.robot.recover_from_errors()

        print(f'Moving to home position: {self.latest_joint_states}')
        home_motion = JointMotion(self.latest_joint_states)
        self.robot.move(home_motion)
        self.robot.relative_dynamics_factor = self.dynamics_factor


        # Set joint limits for more aggressive movement

        print("joint velocity limit: ", self.robot.joint_velocity_limit)
        print("joint acceleration limit: ", self.robot.joint_acceleration_limit)
        print("joint jerk limit: ", self.robot.joint_jerk_limit)

    def joint_state_callback(self, msg):
        try:
            self.latest_joint_states = np.array(msg.position[:7])
            self.joint_states_updated = True
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {e}')

    def smooth_velocity_command(self, raw_velocity):
        """Apply aggressive smoothing to completely eliminate discontinuities"""
        # Step 1: Very conservative velocity change limiting
        velocity_diff = raw_velocity - self.last_velocity_command
        max_change = np.full(7, self.max_velocity_change)
        
        # Clamp velocity changes very aggressively
        clamped_diff = np.clip(velocity_diff, -max_change, max_change)
        smoothed_velocity = self.last_velocity_command + clamped_diff
        
        # Step 2: Apply heavy smoothing with larger history
        self.velocity_history.append(smoothed_velocity)
        
        # Use simple average for maximum smoothing
        if len(self.velocity_history) >= 3:
            smoothed_velocity = np.mean(list(self.velocity_history), axis=0)
        
        # Step 3: Very conservative velocity limiting
        max_velocity = 0.15  # Extremely conservative
        smoothed_velocity = np.clip(smoothed_velocity, -max_velocity, max_velocity)
        
        # Update the last command
        self.last_velocity_command = smoothed_velocity
        return smoothed_velocity

    def timer_callback(self):
        if not self.joint_states_updated:
            return
        
        # Skip every other command to reduce frequency even more
        self.command_count += 1
        if self.command_count % 2 != 0:
            return
            
        try:
            current_joints = np.array(self.robot.current_joint_state.position[:7])
            raw_diff = (self.latest_joint_states - current_joints) * self.velocity_scaling
            
            # Apply aggressive smoothing
            smoothed_diff = self.smooth_velocity_command(raw_diff)
            
            # Skip very small movements
            if np.linalg.norm(smoothed_diff) < 0.005:  # Even smaller threshold
                self.joint_states_updated = False
                return
            
            motion = JointVelocityMotion(
                smoothed_diff, duration=Duration(self.motion_duration_ms),
            )
            self.robot.move(motion, asynchronous=True)
            self.joint_states_updated = False
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')
            self.robot.recover_from_errors()


def main(args=None):
    rclpy.init(args=args)
    franky_relay = FrankyRelay()
    rclpy.spin(franky_relay)
    franky_relay.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()