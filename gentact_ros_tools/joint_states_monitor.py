#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json
import struct
import time

class JointStatesMonitor(Node):
    def __init__(self):
        super().__init__('joint_states_monitor')
        self.get_logger().info('Joint States Monitor with Socket Server started')
        
        # Latest joint state (updated by callback, sent by timer at fixed rate)
        self.latest_joint_state = None
        self.publish_rate_hz = 100.0  # FIXED RATE - no bursts!
        
        # Create socket server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle for low latency
        self.sock.bind(('localhost', 9999))
        self.sock.listen(1)
        self.sock.setblocking(False)  # Non-blocking
        
        self.client = None
        self.get_logger().info(f'Socket server at localhost:9999, publishing at {self.publish_rate_hz} Hz')
        
        # Subscribe to /joint_states (just store latest, don't send immediately)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            1
        )

        # for i in range(7):
        #     self.velocity_subscription = self.create_subscription(
        #         JointState,
        #         f'/panda_joint{i}_velocity/command',
        #         self.velocity_callback_{i},
        #         1
        #     )
        
        # Timer to send at FIXED rate
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        
    def joint_states_callback(self, msg):
        """Just store the latest - don't send here!"""
        # Filter out gripper joints
        arm_positions = []
        for i, name in enumerate(msg.name):
            if 'finger' not in name.lower():
                arm_positions.append(msg.position[i])
        
        # Store latest state (timer will send it)
        self.latest_joint_state = {
            'positions': arm_positions,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }
    
    def timer_callback(self):
        """Send at FIXED rate (100 Hz) - prevents bursts and gaps!"""

        start = time.time()
        # Accept client if needed
        if self.client is None:
            try:
                self.client, addr = self.sock.accept()
                self.client.setblocking(False)
                self.client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info(f'✅ Client connected from {addr}')
            except BlockingIOError:
                return  # No client yet
        # print(f"Time taken: {time.time() - start} seconds")
        
        # Send latest state
        if self.latest_joint_state and self.client:
            try:
                json_str = json.dumps(self.latest_joint_state) + '\n'
                self.client.sendall(json_str.encode())
            except (BrokenPipeError, ConnectionResetError, OSError) as e:
                self.get_logger().warn(f'Client disconnected: {e}')
                try:
                    self.client.close()
                except:
                    pass
                self.client = None


def main(args=None):
    rclpy.init(args=args)
    monitor = JointStatesMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if monitor.client:
            monitor.client.close()
        monitor.sock.close()
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

