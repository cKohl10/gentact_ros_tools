import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import socket
import struct
import threading
import time

class UDPSensorPublisher(Node):
    def __init__(self):
        super().__init__('udp_sensor_publisher')
        
        # Declare parameters
        self.declare_parameter('udp_port', 8888)
        self.declare_parameter('buffer_size', 1024)
        self.declare_parameter('timeout_seconds', 5.0)
        self.declare_parameter('max_devices', 10)  # Maximum number of devices to track
        
        # Get parameters
        self.udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        self.buffer_size = self.get_parameter('buffer_size').get_parameter_value().integer_value
        self.timeout_seconds = self.get_parameter('timeout_seconds').get_parameter_value().double_value
        self.max_devices = self.get_parameter('max_devices').get_parameter_value().integer_value
        
        # Device tracking
        self.device_publishers = {}  # device_id -> publisher
        self.device_last_seen = {}   # device_id -> last_receive_time
        self.device_receive_counts = {}  # device_id -> receive_count
        
        # UDP socket setup
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', self.udp_port))
        self.socket.settimeout(1.0)  # 1 second timeout
        
        # Threading
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        # Create timer for status updates
        self.status_timer = self.create_timer(5.0, self._status_update)
        
        self.get_logger().info(f"UDP Sensor Publisher started on port {self.udp_port}")
        self.get_logger().info(f"Supporting up to {self.max_devices} devices")
    
    def _get_or_create_publisher(self, device_id):
        """Get existing publisher or create new one for device"""
        if device_id not in self.device_publishers:
            if len(self.device_publishers) >= self.max_devices:
                self.get_logger().warn(f"Maximum devices ({self.max_devices}) reached, ignoring device {device_id}")
                return None
            
            # Create new publisher for this device
            topic_name = f'/sensor_raw/device_{device_id:08x}'
            publisher = self.create_publisher(Int32MultiArray, topic_name, 1)
            self.device_publishers[device_id] = publisher
            self.device_last_seen[device_id] = time.time()
            self.device_receive_counts[device_id] = 0
            
            self.get_logger().info(f"New device connected: {device_id:08x} -> {topic_name}")
        
        return self.device_publishers[device_id]
    
    def _receive_loop(self):
        """Main receive loop running in separate thread"""
        while self.running and rclpy.ok():
            try:
                # Receive data
                data, addr = self.socket.recvfrom(self.buffer_size)
                
                # Parse sensor data
                sensor_data = self._parse_sensor_data(data)
                
                if sensor_data is not None:
                    device_id = sensor_data['device_id']
                    
                    # Get or create publisher for this device
                    publisher = self._get_or_create_publisher(device_id)
                    
                    if publisher is not None:
                        # Publish to ROS2
                        self._publish_sensor_data(sensor_data, publisher)
                        
                        # Update status
                        self.device_last_seen[device_id] = time.time()
                        self.device_receive_counts[device_id] += 1
                        
                        # Log received data
                        self.get_logger().debug(
                            f"Received from {addr}: device_id={device_id:08x}, "
                            f"sensors={sensor_data['sensor_values']}, "
                            f"raw_size={len(sensor_data['raw_data'])}"
                        )
                    
            except socket.timeout:
                # Timeout is expected, continue
                continue
            except Exception as e:
                self.get_logger().error(f"Error in receive loop: {e}")
                continue
    
    def _parse_sensor_data(self, data):
        """Parse binary sensor data from ESP32"""
        try:
            # Expected structure: device_id(4) + num_sensors(4) + sensor_values(4*num_sensors)
            if len(data) < 12:  # Minimum size (4+4+4)
                self.get_logger().warn(f"Received data too small: {len(data)} bytes")
                return None
            
            # Unpack binary data
            device_id = struct.unpack('<I', data[0:4])[0]
            num_sensors = struct.unpack('<I', data[4:8])[0]
            
            # Validate number of sensors
            if num_sensors > 8 or num_sensors <= 0:
                self.get_logger().warn(f"Invalid sensor count: {num_sensors}")
                return None
            
            # Validate device_id
            if device_id == 0:
                self.get_logger().warn("Invalid device_id: 0")
                return None
            
            # Calculate expected data size
            expected_size = 8 + (num_sensors * 4)  # header + sensor values
            if len(data) < expected_size:
                self.get_logger().warn(f"Data size mismatch: expected {expected_size}, got {len(data)}")
                return None
            
            # Parse sensor values (each long is 4 bytes)
            sensor_values = []
            for i in range(num_sensors):
                start_idx = 8 + (i * 4)
                end_idx = start_idx + 4
                if end_idx <= len(data):
                    # Parse as long (32-bit integer) from ESP32
                    value = struct.unpack('<l', data[start_idx:end_idx])[0]
                    sensor_values.append(value)
            
            return {
                'device_id': device_id,
                'num_sensors': num_sensors,
                'sensor_values': sensor_values,
                'raw_data': data
            }
            
        except struct.error as e:
            self.get_logger().error(f"Failed to parse sensor data: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error parsing data: {e}")
            return None
    
    def _publish_sensor_data(self, sensor_data, publisher):
        """Publish sensor data as ROS2 message"""
        try:
            # Create Int32MultiArray message
            msg = Int32MultiArray()
            
            # Convert and clamp sensor values to valid int32 range
            clamped_values = []
            for value in sensor_data['sensor_values']:
                # Clamp to int32 range [-2147483648, 2147483647]
                clamped_value = max(-2147483648, min(2147483647, int(value)))
                clamped_values.append(clamped_value)
            
            msg.data = clamped_values
            
            # Publish
            publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing sensor data: {e}")
    
    def _status_update(self):
        """Periodic status update"""
        current_time = time.time()
        active_devices = 0
        total_packets = 0
        
        # Check each device
        for device_id in list(self.device_last_seen.keys()):
            time_since_last = current_time - self.device_last_seen[device_id]
            packet_count = self.device_receive_counts.get(device_id, 0)
            total_packets += packet_count
            
            if time_since_last > self.timeout_seconds:
                self.get_logger().warn(
                    f"Device {device_id:08x}: No data for {time_since_last:.1f}s "
                    f"(total packets: {packet_count})"
                )
            else:
                active_devices += 1
                self.get_logger().info(
                    f"Device {device_id:08x}: {packet_count} packets, "
                    f"last: {time_since_last:.1f}s ago"
                )
        
        if active_devices == 0:
            self.get_logger().warn(f"No active devices. Total packets received: {total_packets}")
        else:
            self.get_logger().info(
                f"Active devices: {active_devices}/{len(self.device_publishers)}, "
                f"Total packets: {total_packets}"
            )
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.running = False
        if hasattr(self, 'socket'):
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    publisher = UDPSensorPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 