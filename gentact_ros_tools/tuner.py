import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class TunerPublisher(Node):
    def __init__(self, value):
        super().__init__('tuner_publisher')
        
        self.publisher = self.create_publisher(Float64, '/sensor_tuning', 10)
        
        # Create and publish the message
        msg = Float64()
        msg.data = float(value)
        
        # Give some time for the publisher to connect
        self.create_timer(0.1, lambda: self.publish_and_shutdown(msg))
        
        self.get_logger().info(f"Publishing tuning value: {value}")
    
    def publish_and_shutdown(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info(f"Published tuning alpha: {msg.data}")
        # Shutdown after publishing
        rclpy.shutdown()


def main(args=None):
    # Check command line arguments
    if len(sys.argv) != 2:
        print("Usage: ros2 run gentact_ros_tools tuner <number>")
        print("Example: ros2 run gentact_ros_tools tuner 0.001")
        sys.exit(1)
    
    try:
        value = float(sys.argv[1])
    except ValueError:
        print(f"Error: '{sys.argv[1]}' is not a valid number")
        sys.exit(1)
    
    rclpy.init(args=args)
    
    try:
        tuner = TunerPublisher(value)
        rclpy.spin(tuner)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main() 