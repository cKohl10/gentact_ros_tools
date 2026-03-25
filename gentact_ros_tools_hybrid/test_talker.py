#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TestTalker(Node):
    def __init__(self):
        super().__init__('test_talker')
        self.publisher = self.create_publisher(String, '/test_topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish at 2 Hz
        self.count = 0
        self.get_logger().info('🔊 Test Talker started - publishing to /test_topic')
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from Docker! Message #{self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    talker = TestTalker()
    
    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()












