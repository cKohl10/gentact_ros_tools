#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import numpy as np
from collections import deque
import sys
import os

# Add libfranka to Python path
sys.path.append('/home/caleb/libfranka/build')
sys.path.append('/home/caleb/libfranka')

try:
    from franka import *
except ImportError as e:
    print(f"Error importing franky: {e}")
    print("Make sure libfranka is built and in the Python path")
    sys.exit(1)

class FrankaROS2Controller(Node):
    def __init__(self):
        super().__init__('franka_ros2_controller')
        self.get_logger().info('Franka ROS2 Controller node initialized')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.0.198')
        self.declare_parameter('arm_id', 'fr3')
        self.declare_parameter('velocity_scaling', 3.0)
        self.declare_parameter('dynamics_factor', 0.3)
        self.declare_parameter('motion_duration_ms', 100)
        self.declare_parameter('update_rate_hz', 20.0)
        
        # Get parameter values
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.arm_id = self.get_parameter('arm_id').get_parameter_value().string_value
        self.velocity_scaling = self.get_parameter('velocity_scaling').get_parameter_value().double_value
        self.dynamics_factor = self.get_parameter('dynamics_factor').get_parameter_value().double_value
        self.motion_duration_ms = self.get_parameter('motion_duration_ms').get_parameter_value().integer_value
        self.update_rate_hz = self.get_parameter('update_rate_hz').get_parameter_value().double_value
        
        # Store latest joint state data
        self.latest_joint_states = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.joint_states_updated = False
        
        # Velocity smoothing
        self.velocity_history = deque(maxlen=5)
        self.last_velocity_command = np.zeros(7)
        self.max_velocity_change = 0.05
        self.command_count = 0
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState, 
            f'joint_states_{self.arm_id}', 
            self.joint_state_callback, 
            1
        )
        
        # Create timer
        self.timer = self.create_timer(1.0/self.update_rate_hz, self.timer_callback)
        
        # Initialize robot connection
        self.get_logger().info(f'Connecting to robot at {self.robot_ip}...')
        try:
            self.robot = Robot(self.robot_ip)
            self.robot.relative_dynamics_factor = self.dynamics_factor
            self.robot.recover_from_errors()
            self.get_logger().info('Robot connected successfully!')
            
            # Move to home position
            self.move_to_home()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot: {e}')
            raise

    def joint_state_callback(self, msg):
        try:
            # Extract joint positions (assuming they're in the correct order)
            if len(msg.position) >= 7:
                self.latest_joint_states = np.array(msg.position[:7])
                self.joint_states_updated = True
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {e}')

    def move_to_home(self):
        """Move robot to home position"""
        home_trajectory = JointTrajectory()
        home_trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = self.latest_joint_states.tolist()
        point.time_from_start.sec = 3  # 3 seconds to reach home
        home_trajectory.points = [point]
        
        self.trajectory_pub.publish(home_trajectory)
        self.get_logger().info(f'Moving to home position: {self.latest_joint_states}')

    def smooth_velocity_command(self, raw_velocity):
        """Apply smoothing to velocity commands"""
        # Limit velocity changes
        velocity_diff = raw_velocity - self.last_velocity_command
        max_change = np.full(7, self.max_velocity_change)
        clamped_diff = np.clip(velocity_diff, -max_change, max_change)
        smoothed_velocity = self.last_velocity_command + clamped_diff
        
        # Apply smoothing with history
        self.velocity_history.append(smoothed_velocity)
        if len(self.velocity_history) >= 3:
            smoothed_velocity = np.mean(list(self.velocity_history), axis=0)
        
        # Limit maximum velocity
        max_velocity = 0.5
        smoothed_velocity = np.clip(smoothed_velocity, -max_velocity, max_velocity)
        
        self.last_velocity_command = smoothed_velocity
        return smoothed_velocity

    def timer_callback(self):
        if not self.joint_states_updated:
            return
            
        try:
            # Create trajectory message
            trajectory = JointTrajectory()
            trajectory.joint_names = self.joint_names
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = self.latest_joint_states.tolist()
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(1000000000 / self.update_rate_hz)  # 1/rate seconds
            
            trajectory.points = [point]
            
            # Publish trajectory
            self.trajectory_pub.publish(trajectory)
            self.joint_states_updated = False
            
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    controller = FrankaROS2Controller()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
