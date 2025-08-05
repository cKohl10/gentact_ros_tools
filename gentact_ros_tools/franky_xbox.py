#! /usr/bin/env python3

import sys
import os

# Try to find and use the virtual environment
def setup_venv():
    venv_path = "/home/caleb/avoidance_controller/.venv"
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

    xbox = "/home/caleb/avoidance_controller/franka_xbox_control_white_pc/run_robot_control.py"

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
import time
from sensor_msgs.msg import JointState
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation
from dataclasses import dataclass
import threading
from inputs import get_gamepad
import signal


def truncate_float(x, decimal_places=1):
    # print(x)
    factor = 10**decimal_places
    truncated_x = int(x * factor) / factor
    # print(truncated_x)
    return truncated_x


class GameController(object):
    # These values are different for different controllers.
    MAX_TRIG_VAL = 128.0
    MAX_JOY_VAL = 32767.0

    def __init__(self):
        self.LeftJoystickY = 0.0
        self.LeftJoystickX = 0.0
        self.RightJoystickY = 0.0
        self.RightJoystickX = 0.0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.Square = 0
        self.Triangle = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.HorizontalDPad = 0
        self.VerticalDPad = 0

        self._monitor_thread = threading.Thread(
            target=self._monitor_controller, args=()
        )
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def get_robot_inputs_from_controller(self):
        position_multiplier = 10.0 / 100.0
        angle_multiplier = 1.0
        left_x = truncate_float(self.LeftJoystickY) * position_multiplier
        left_y = truncate_float(self.LeftJoystickX) * position_multiplier
        right_z = -truncate_float(self.RightJoystickY) * position_multiplier

        roll = self.VerticalDPad
        pitch = self.HorizontalDPad
        pitch *= angle_multiplier
        roll *= angle_multiplier
        yaw_neg = self.LeftBumper * angle_multiplier
        yaw_pos = self.RightBumper * angle_multiplier
        yaw = yaw_pos - yaw_neg

        gripper = 1 if self.RightTrigger > 0.5 else -1 if self.LeftTrigger > 0.5 else 0

        square_btn = self.Square == 1
        triangle_btn = self.Triangle == 1
        # print(f"Left: {left_x:.2f}, {left_y:.2f}, Right Z: {right_z:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}, Gripper: {gripper:.2f}, X: {square_btn}, Y: {triangle_btn}")
        return RobotInputs([left_x, left_y, right_z, roll, pitch, yaw, gripper], square_btn, triangle_btn)

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                if event.code == "ABS_Y":
                    self.LeftJoystickY = (
                        event.state
                    ) / GameController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == "ABS_X":
                    self.LeftJoystickX = (
                        event.state
                    ) / GameController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == "ABS_RY":
                    self.RightJoystickY = (
                        event.state
                    ) / GameController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == "ABS_RX":
                    self.RightJoystickX = (
                        event.state
                    ) / GameController.MAX_JOY_VAL  # normalize between -1 and 1
                elif event.code == "ABS_Z":
                    self.LeftTrigger = (
                        event.state / GameController.MAX_TRIG_VAL
                    )  # normalize between 0 and 1
                elif event.code == "ABS_RZ":
                    self.RightTrigger = (
                        event.state / GameController.MAX_TRIG_VAL
                    )  # normalize between 0 and 1
                elif event.code == "BTN_TL":
                    self.LeftBumper = event.state
                elif event.code == "BTN_TR":
                    self.RightBumper = event.state
                elif event.code == "BTN_SOUTH":
                    self.A = event.state
                elif event.code == "BTN_NORTH":
                    self.Triangle = event.state  # On some controllers, this is switched with BTN_WEST
                elif event.code == "BTN_WEST":
                    self.Square = event.state  # On some controllers, this is switched with BTN_NORTH
                elif event.code == "BTN_EAST":
                    self.B = event.state
                elif event.code == "BTN_THUMBL":
                    self.LeftThumb = event.state
                elif event.code == "BTN_THUMBR":
                    self.RightThumb = event.state
                elif event.code == "BTN_SELECT":
                    self.Back = event.state
                elif event.code == "BTN_START":
                    self.Start = event.state
                elif event.code == "ABS_HAT0X":
                    self.HorizontalDPad = event.state
                elif event.code == "ABS_HAT0Y":
                    self.VerticalDPad = event.state


@dataclass
class RobotInputs:
    left_x: float
    left_y: float
    right_z: float
    roll: float
    pitch: float
    yaw: float
    gripper: float
    square_btn: bool
    triangle_btn: bool

    def __init__(self, array=None, square_btn=None, triangle_btn=None):
        if array is not None:
            self.left_x = array[0]
            self.left_y = array[1]
            self.right_z = array[2]
            self.roll = array[3]
            self.pitch = array[4]
            self.yaw = array[5]
            self.gripper = array[6]
            self.square_btn = square_btn
            self.triangle_btn = triangle_btn
        else:
            # Default values when no array is provided
            self.left_x = 0.0
            self.left_y = 0.0
            self.right_z = 0.0
            self.roll = 0.0
            self.pitch = 0.0
            self.yaw = 0.0
            self.gripper = 0.0
            self.square_btn = False
            self.triangle_btn = False


class FrankaRobot(object):
    def __init__(self, robot_ip="192.168.0.198"):
        self.robot = Robot(robot_ip)
        # self.gripper = Gripper(robot_ip)
        self.gripper_speed = 0.05  # m/s
        # self.min_gripper_pos_change = self.gripper.max_width / 10.0
        self.robot.recover_from_errors()
        # This controls the max percentage of jerk, acceleration, and velocity allowed in a motion.
        # A value of 0.05 means 5% of the maximum allowed.
        self.robot.relative_dynamics_factor = 0.05
        self.home = JointMotion([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

    def move_home(self):
        self.robot.move(self.home)
        # self.gripper.move(self.gripper.max_width, self.gripper_speed)
        # self.gripper.move(0, self.gripper_speed)

    def print_q(self):
        print(f"{self.robot.current_joint_positions=}")

    def rotate_to_zero(self):
        quat = Rotation.from_euler("xyz", [0, 0, 0]).as_quat()
        curr_pose = self.robot.current_pose
        self.robot.move(
            CartesianMotion(
                RobotPose(Affine(curr_pose.end_effector_pose.translation, quat))
            )
        )

    def move_velocity_inputs(self, robot_inputs: RobotInputs, duration_ms=500):
        linear_velocity = [robot_inputs.left_x,
                           robot_inputs.left_y, robot_inputs.right_z]
        angular_velocity = [robot_inputs.roll,
                            robot_inputs.pitch, robot_inputs.yaw]
    

        self.move_velocity_array(
            linear_velocity, angular_velocity, robot_inputs.gripper, duration_ms=duration_ms)

    def move_velocity_array(self, linear_velocity, angular_velocity, gripper, duration_ms=500):

        self.robot.move(
            CartesianVelocityMotion(
                Twist(
                    linear_velocity=linear_velocity,
                    angular_velocity=angular_velocity,
                ),
                duration=Duration(duration_ms),
            ),
            asynchronous=True,
        )

        if gripper > 0.5:
            self.gripper_grasp_open()
        if gripper < -0.5:
            self.gripper_grasp_close()

    def open_gripper(self):
        new_position = min(self.gripper.width +
                           self.min_gripper_pos_change, self.gripper.max_width)
        self.gripper.move_async(speed=self.gripper_speed, width=new_position)

    def close_gripper(self):
        new_position = max(self.gripper.width - self.min_gripper_pos_change, 0)
        self.gripper.move_async(speed=self.gripper_speed, width=new_position)

    def gripper_grasp_close(self, force=20):
        self.gripper.grasp(width=0.0, speed=self.gripper_speed, force=force)

    def gripper_grasp_open(self, force=20):
        self.gripper.open(self.gripper_speed)

    def quaternion_array_to_rpy(self, quaternion, degrees=False):
        """
        Convert array of quaternions to RPY angles.

        Args:
            quaternions: Array of shape (N, 4) with quaternions as [x, y, z, w]
            degrees: If True, return angles in degrees; if False, radians

        Returns:
            array: Shape (N, 3) with [roll, pitch, yaw] for each quaternion
        """
        rot = Rotation.from_quat(quaternion)
        return rot.as_euler('xyz', degrees=degrees)

    def get_ee_state(self):
        start = time.time()
        ee_pos_q = self.robot.current_cartesian_state.pose.end_effector_pose.quaternion
        print(f"ee_pos_q {time.time() - start}")
        start = time.time()
        ee_pos_rpy = self.quaternion_array_to_rpy(ee_pos_q)
        ee_pos_t = self.robot.current_cartesian_state.pose.end_effector_pose.translation
        print(f"ee_pos_t {time.time() - start}")
        gpos = self.gripper.width

        obs_data = {
            "ee_pos_t": ee_pos_t,
            "ee_pos_rpy": ee_pos_rpy,
            "gpos": np.array([gpos])
        }

        return obs_data
    


class FrankyXbox(Node):
    def __init__(self):
        super().__init__('franky_xbox_controller')
        self.get_logger().info('Franky Xbox Controller node initialized')
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Initialize robot control components
        self.running = True
        self.joy = GameController()
        self.robot = FrankaRobot()
        self.robot.robot.recover_from_errors()

        # Control loop parameters
        self.target_frequency = 100  # Hz
        self.timer_period = 1.0 / self.target_frequency  # seconds
        
        # Initialize robot to home position
        self.initialized = False
        
        # Set up signal handler for graceful shutdown

        # Create timer for control loop
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        
        self.get_logger().info('Beginning control loop (Press Ctrl+C to exit gracefully)')

    '''
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        self.get_logger().info(f"Shutdown signal {signum} received. Stopping robot gracefully...")
        self.running = False

        # Don't call shutdown here as it will be called in the main function's finally block
    '''
      
    def control_loop(self):
        """Main control loop with proper error handling and timing"""
        if not self.running:
            return
            
        try:
            # Initialize robot to home position on first run
            if not self.initialized:
                self.robot.move_home()
                self.initialized = True
                self.get_logger().info("Robot moved to home position")

            # Print current joint positions
            # self.robot.print_q()
            joint_positions = self.robot.robot.current_joint_state.position[:7]
            
            # Create proper JointState message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = [f'panda_joint{i+1}' for i in range(7)]
            joint_state_msg.position = joint_positions.tolist() if hasattr(joint_positions, 'tolist') else list(joint_positions)
            joint_state_msg.velocity = [0.0] * 7  # Add zero velocities
            joint_state_msg.effort = [0.0] * 7    # Add zero efforts
            
            self.joint_pub.publish(joint_state_msg)

            try:
                # Get controller inputs
                robot_inputs = self.joy.get_robot_inputs_from_controller()
                # self.get_logger().debug(f"Controller inputs: {robot_inputs}")

                # Send commands to robot
                self.robot.move_velocity_inputs(robot_inputs)

            except Exception as e:
                self.get_logger().error(f"Exception occurred while moving robot: {e}")
                try:
                    self.robot.robot.recover_from_errors()
                    self.get_logger().info("Robot recovered from errors")
                except Exception as recovery_error:
                    self.get_logger().error(f"Failed to recover from errors: {recovery_error}")
                    self.running = False
                    return

        except Exception as e:
            self.get_logger().error(f"Fatal error in control loop: {e}")
            self.running = False

    def shutdown(self):
        """Clean shutdown procedure"""
        self.get_logger().info("Shutting down robot controller...")
        
        # Stop the control loop first
        self.running = False
        
        try:
            # Stop any ongoing robot motion
            self.robot.robot.stop()
            self.get_logger().info("Robot motion stopped")
            
            # Wait a moment for any async operations to complete
            time.sleep(0.5)
            
            # Try to recover from any errors that might have occurred during shutdown
            self.robot.robot.recover_from_errors()
            self.get_logger().info("Robot recovered from errors")
            
        except Exception as e:
            self.get_logger().error(f"Error during robot shutdown: {e}")
        
        # Clean up the game controller
        '''
        try:
            if hasattr(self, 'joy') and self.joy:
                # Stop the controller monitoring thread
                self.joy._monitor_thread.join(timeout=1.0)
                self.get_logger().info("Game controller thread stopped")
        except Exception as e:
            self.get_logger().error(f"Error stopping game controller: {e}")
        '''
            
        self.get_logger().info("Shutdown complete")


def main(args=None):
    rclpy.init(args=args)
    franky_xbox = FrankyXbox()
    
    rclpy.spin(franky_xbox)
    franky_xbox.shutdown()
    franky_xbox.destroy_node()
    rclpy.shutdown()
    
    franky_xbox.get_logger().info("Cleanup complete")

if __name__ == "__main__":
    main()