import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import numpy as np


class SensorPublisher(Node):
    def __init__(self):
        super().__init__("sensor_publisher")
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('num_sensors', 3)
        self.declare_parameter('publish_rate', 30.0)  # Hz
        self.declare_parameter('startup_timeout', 3.0)  # seconds
        self.declare_parameter('calibration_duration', 3.0)  # seconds
        self.declare_parameter('skin_name', '')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.startup_timeout = self.get_parameter('startup_timeout').get_parameter_value().double_value
        self.calibration_duration = self.get_parameter('calibration_duration').get_parameter_value().double_value
        self.skin_name = self.get_parameter('skin_name').get_parameter_value().string_value
        self.last_data = []
        self.baseline_data = None
        
        # Initialize publisher
        if self.skin_name == '':
            self.publisher = self.create_publisher(
                Int32MultiArray, 
                '/sensor_raw', 
                1
            )
            self.baseline_pub = self.create_publisher(
                Int32MultiArray,
                '/sensor_baseline',
                1
            )
        else:
            self.publisher = self.create_publisher(
                Int32MultiArray, 
                f'/sensor_raw_{self.skin_name}', 
                1
            )
            self.baseline_pub = self.create_publisher(
                Int32MultiArray,
                f'/sensor_baseline_{self.skin_name}',
                1
            )
        
        # Initialize serial connection
        try:
            self.serial = serial.Serial(self.serial_port, 9600)
            self.get_logger().info(f"Connected to serial port: {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            self.serial = None
        
        # Wait for first valid reading before starting
        self.wait_for_valid_reading()

        # Perform calibration to get baseline values
        self.perform_calibration()
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_sensor_data)
        
        self.get_logger().info("Sensor publisher initialized and ready")

    def format_sensor_data(self, data):
        try:
            data_str = data.decode('utf-8').strip()
            first_element = data_str.split(',')[0].strip()
            if first_element and 'S' in first_element:
                return np.array([int(x.strip()) for x in data_str.split(',')[1:] if x.strip()])
            return np.array([int(x.strip()) for x in data_str.split(',') if x.strip()]) 
        except Exception as e:
            self.get_logger().error(f"Error formatting sensor data: {e}")
            return []

    
    def perform_calibration(self):
        """Collect sensor data for 3 seconds and calculate baseline average"""
        if self.serial is None:
            self.get_logger().error("No serial connection available for calibration.")
            return
            
        self.get_logger().info(f"Starting calibration for {self.calibration_duration} seconds...")
        
        calibration_data = []
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            current_time = self.get_clock().now()
            elapsed_time = (current_time - start_time).nanoseconds / 1e9
            
            if elapsed_time >= self.calibration_duration:
                break
                
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.readline()
                    read_numbers = self.format_sensor_data(data)
                    if len(read_numbers) >= self.num_sensors:
                        calibration_data.append(read_numbers[:self.num_sensors])
                        
            except serial.SerialException as e:
                self.get_logger().warn(f"Serial error during calibration: {e}")
                continue
            except Exception as e:
                self.get_logger().warn(f"Unexpected error during calibration: {e}")
                continue
        
        if calibration_data:
            # Calculate average of all collected data
            calibration_array = np.array(calibration_data)
            self.baseline_data = np.mean(calibration_array, axis=0).astype(int)
            
            self.get_logger().info(f"Calibration complete. Baseline values: {self.baseline_data}")
            self.get_logger().info(f"Collected {len(calibration_data)} samples during calibration")
            
            # Publish baseline data
            baseline_msg = Int32MultiArray()
            baseline_msg.data = self.baseline_data.tolist()
            self.baseline_pub.publish(baseline_msg)
            self.get_logger().info("Published baseline calibration data")
        else:
            self.get_logger().error("No valid data collected during calibration")
            self.baseline_data = np.zeros(self.num_sensors, dtype=int)
    
    def wait_for_valid_reading(self):
        """Wait until we receive a valid sensor reading"""
        if self.serial is None:
            self.get_logger().error("No serial connection available. Shutting down.")
            raise RuntimeError("Failed to connect to serial port. Cannot get sensor readings.")
            
        self.get_logger().info("Waiting for first valid sensor reading...")
        
        start_time = self.get_clock().now()
        
        while rclpy.ok():
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > self.startup_timeout:
                self.get_logger().error(f"Timeout: No valid sensor reading received within {self.startup_timeout} seconds. Shutting down.")
                raise TimeoutError(f"Sensor reading timeout after {self.startup_timeout}s")

            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.readline()
                    read_numbers = self.format_sensor_data(data)
                    
                    if len(read_numbers) >= self.num_sensors:
                        self.last_data = read_numbers[:self.num_sensors]
                        self.get_logger().info(f"Received first valid reading: {self.last_data}")
                        return
                        
            except serial.SerialException as e:
                self.get_logger().warn(f"Serial error while waiting: {e}")
                continue
            except Exception as e:
                self.get_logger().warn(f"Unexpected error while waiting: {e}")
                continue
    
    def read_serial(self):
        """Read data from serial port and parse sensor values"""
        if self.serial is None:
            return self.last_data
            
        try:
            # Check if data is available
            if self.serial.in_waiting > 0:
                data = self.serial.readline()
                read_numbers = self.format_sensor_data(data)
                if len(read_numbers) >= self.num_sensors:
                    self.last_data = read_numbers[:self.num_sensors]
                    
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial read error: {e}")
        except Exception as e:
            self.get_logger().warn(f"Unexpected error reading serial: {e}")
        
        return self.last_data
    
    def publish_sensor_data(self):
        """Read sensor data and publish as ROS2 message"""
        try:
            sensor_values = self.read_serial()
            
            # Create message
            msg = Int32MultiArray()
            msg.data = sensor_values.tolist()
            self.publisher.publish(msg)
            
            # Log for debugging
            self.get_logger().debug(f"Published sensor data: {sensor_values}")
        except Exception as e:
            self.get_logger().error(f"Error in publish_sensor_data: {e}")
    
    def __del__(self):
        """Cleanup serial connection"""
        if hasattr(self, 'serial') and self.serial is not None:
            self.serial.close()


def main(args=None):
    rclpy.init(args=args)
    
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 