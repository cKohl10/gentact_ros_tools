#!/tmp/franky_repo/venv-franky/bin/python3
"""
Socket receiver for joint states - runs with Python 3.10 and franky
This receives joint positions from the ROS2 socket publisher
and controls the real robot with franky library
"""

import socket
import json
import sys
import numpy as np
from collections import deque
import time

# Import franky library
try:
    from franky import Robot, JointVelocityMotion, Duration
    print("✅ franky library imported successfully")
except ImportError as e:
    print(f"❌ ERROR: Could not import franky: {e}")
    print(f"Python: {sys.executable}")
    print(f"Version: {sys.version}")
    sys.exit(1)


class FrankySocketController:
    def __init__(self, robot_ip="192.168.0.100"):
        print("=" * 60)
        print("Franky Socket Controller (Python 3.10)")
        print("=" * 60)
        print(f"Python version: {sys.version}")
        
        # Controller parameters (same as franky_relay)
        self.velocity_scaling = 3.0
        self.dynamics_factor = 0.3
        self.motion_duration_ms = 200
        self.update_rate_hz = 50.0
        
        print(f"Controller gains:")
        print(f"  velocity_scaling = {self.velocity_scaling}")
        print(f"  dynamics_factor = {self.dynamics_factor}")
        print(f"  duration = {self.motion_duration_ms}ms")
        print(f"  rate = {self.update_rate_hz}Hz")
        
        # Velocity smoothing
        self.velocity_history = deque(maxlen=5)
        self.last_velocity_command = np.zeros(7)
        self.max_velocity_change = 0.05
        self.command_count = 0
        
        # Latest joint states from socket
        self.latest_joint_states = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
        self.joint_states_updated = False
        
        # Connect to robot
        print(f"\nConnecting to robot at {robot_ip}...")
        try:
            self.robot = Robot(robot_ip)
            self.robot.relative_dynamics_factor = self.dynamics_factor
            self.robot.recover_from_errors()
            print("✅ Robot connected!")
            
            print(f"\nRobot limits:")
            print(f"  Joint velocity limit: {self.robot.joint_velocity_limit}")
            print(f"  Joint acceleration limit: {self.robot.joint_acceleration_limit}")
            print(f"  Joint jerk limit: {self.robot.joint_jerk_limit}")
            
        except Exception as e:
            print(f"❌ ERROR: Could not connect to robot: {e}")
            raise
    
    def smooth_velocity_command(self, raw_velocity):
        """Apply smoothing to velocity commands (same logic as franky_relay)"""
        # Step 1: Conservative velocity change limiting
        velocity_diff = raw_velocity - self.last_velocity_command
        max_change = np.full(7, self.max_velocity_change)
        clamped_diff = np.clip(velocity_diff, -max_change, max_change)
        smoothed_velocity = self.last_velocity_command + clamped_diff
        
        # Step 2: Apply smoothing with history
        self.velocity_history.append(smoothed_velocity)
        if len(self.velocity_history) >= 3:
            smoothed_velocity = np.mean(list(self.velocity_history), axis=0)
        
        # Step 3: Velocity limiting
        max_velocity = 0.5
        smoothed_velocity = np.clip(smoothed_velocity, -max_velocity, max_velocity)
        
        self.last_velocity_command = smoothed_velocity
        return smoothed_velocity
    
    def update_desired_position(self, positions):
        """Update the desired joint positions from socket"""
        if len(positions) == 7:
            self.latest_joint_states = np.array(positions)
            self.joint_states_updated = True
            # Minimal debug - only print every 50 updates
            # if self.command_count % 50 == 0:
            #     print(f"[UPDATE] Desired: {[f'{p:.3f}' for p in positions]}")
        else:
            print(f"⚠️  WARNING: Expected 7 joints, got {len(positions)}")
    
    def control_step(self):
        """Execute one control step (same logic as franky_relay timer_callback)"""
        # if not self.joint_states_updated:
        #     return
        
        # # Skip every other command to reduce frequency
        # self.command_count += 1
        # if self.command_count % 2 != 0:
        #     return
        
        try:
            # Get current robot position
            current_joints = np.array(self.robot.current_joint_state.position[:7])
            
            # Calculate velocity command
            raw_diff = (self.latest_joint_states - current_joints) * self.velocity_scaling
            smoothed_diff = self.smooth_velocity_command(raw_diff)
            
            # Skip very small movements
            # if np.linalg.norm(smoothed_diff) < 0.005:
            #     self.joint_states_updated = False
            #     return
            
            # Minimal debug - only print every 20 commands
            # if self.command_count % 20 == 0:
            #     print(f"[CONTROL #{self.command_count}]")
            #     print(f"  Current:  {[f'{p:.3f}' for p in current_joints]}")
            #     print(f"  Desired:  {[f'{p:.3f}' for p in self.latest_joint_states]}")
            #     print(f"  Velocity: {[f'{v:.3f}' for v in smoothed_diff]} (norm: {np.linalg.norm(smoothed_diff):.4f})")
            
            # Send motion to robot
            motion = JointVelocityMotion(
                smoothed_diff.tolist(),
                duration=Duration(self.motion_duration_ms)
            )
            self.robot.move(motion, asynchronous=True)
            # self.joint_states_updated = False
            
        except Exception as e:
            print(f"❌ ERROR in control step: {e}")
            self.robot.recover_from_errors()


def main():
    # Connect to robot first
    try:
        controller = FrankySocketController(robot_ip="192.168.0.100")
    except Exception as e:
        print(f"Failed to initialize controller: {e}")
        return 1
    
    # Connect to socket
    print("\nConnecting to socket server at localhost:9999...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm for low latency
    sock.settimeout(0.01)  # Very short timeout for non-blocking behavior
    
    try:
        sock.connect(('localhost', 9999))
        print("✅ Connected to joint states publisher!")
        print("=" * 60)
        print("Starting control loop...")
        print("=" * 60)
        
        buffer = ""
        last_control_time = time.time()
        control_period = 1.0 / controller.update_rate_hz
        
        while True:
            start = time.time()
            # Receive data from socket (non-blocking with timeout)
            try:
                data = sock.recv(4096).decode()
                if not data:
                    print("Connection closed by server")
                    break
                
                buffer += data
                
                # Process complete messages
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            joint_data = json.loads(line)
                            positions = joint_data['positions']
                            timestamp = joint_data['timestamp']
                            
                            # Update desired position
                            controller.update_desired_position(positions)
                            
                        except json.JSONDecodeError as e:
                            print(f"⚠️  JSON decode error: {e}")
            
            except socket.timeout:
                pass  # No data available, continue
            except BlockingIOError:
                pass  # No data available

            
            # Execute control at fixed rate
            current_time = time.time()
            if current_time - last_control_time >= control_period:
                controller.control_step()
                last_control_time = current_time

                # print(f"a Time taken: {time.time() - start} seconds")
            # print(f"b Time taken: {time.time() - start} seconds")
            # No sleep - run as fast as possible for minimal latency
                        
    except ConnectionRefusedError:
        print("❌ ERROR: Could not connect to socket server")
        print("Make sure joint_states_monitor is running first!")
        return 1
    except KeyboardInterrupt:
        print("\n\n🛑 Shutting down...")
    finally:
        sock.close()
        print("Socket closed")
        print("Robot control stopped")
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

