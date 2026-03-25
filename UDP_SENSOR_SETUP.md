# UDP Sensor System - Phase 1 Implementation

This implementation provides wireless sensor data transmission from ESP32 devices to ROS2 using UDP with a simple binary float format for efficient compression.

## System Overview

- **ESP32 Side**: Reads capacitive sensors and sends data via UDP
- **ROS2 Side**: Receives UDP data and publishes as ROS2 messages
- **Data Format**: Binary structure with device_id, timestamp, sensor count, and float values

## ESP32 Setup

### Hardware Requirements
- ESP32 development board
- Capacitive sensors (connected to GPIO pins 4, 5, 18)
- WiFi network access

### Software Setup
1. Install Arduino IDE with ESP32 board support
2. Open `esp32_sensor_sender.ino`
3. Update WiFi credentials:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
4. Update target IP address:
   ```cpp
   const char* udpAddress = "192.168.1.100";  // Your ROS2 machine IP
   ```
5. Upload to ESP32

### Configuration
- **Sensor Pins**: GPIO 4, 6, 8 (CapacitiveSensor send/receive pairs)
- **Update Rate**: 100 Hz (modify `delay(10)` in loop)
- **Device ID**: 0x00000001 (modify `sensorData.device_id`)
- **CapacitiveSensor**: Uses 10M resistors between send pin (4) and receive pins (2, 6, 8)

## ROS2 Setup

### Build and Install
```bash
# In your ROS2 workspace
colcon build --packages-select gentact_ros_tools_hybrid
source install/setup.bash
```

### Run UDP Publisher
```bash
# Option 1: Direct execution
ros2 run gentact_ros_tools_hybrid udp_sensor_publisher

# Option 2: Using launch file
ros2 launch gentact_ros_tools_hybrid udp_sensor_publisher.launch.py
```

### Parameters
- `udp_port`: 8888 (default)
- `buffer_size`: 1024 bytes (default)
- `timeout_seconds`: 5.0 seconds (default)

## Testing

### Local Testing (without ESP32)
1. Start the UDP publisher:
   ```bash
   ros2 run gentact_ros_tools_hybrid udp_sensor_publisher
   ```

2. In another terminal, run the test sender:
   ```bash
   python3 test_udp_sender.py
   ```

3. Monitor the sensor data:
   ```bash
   ros2 topic echo /sensor_raw
   ```

### With Real ESP32
1. Ensure ESP32 and ROS2 machine are on same WiFi network
2. Update ESP32 code with correct WiFi credentials and target IP
3. Upload ESP32 code and power on
4. Start ROS2 receiver
5. Monitor data: `ros2 topic echo /sensor_raw`

## Data Format

### Binary Structure (40 bytes total)
```
Offset  Size    Field
0-3     4       device_id (uint32)
4-7     4       num_sensors (uint32)
8-39    32      sensor_values (8 x float32)
```

### Example Data
- Device ID: 1
- Sensor Count: 3
- Sensor Values: [1234, 5678, 9012] (raw capacitive sensor values)

## Troubleshooting

### ESP32 Issues
- **WiFi Connection**: Check SSID/password, ensure network is 2.4GHz
- **UDP Send**: Verify target IP address is correct
- **Sensor Reading**: Check GPIO pin connections

### ROS2 Issues
- **No Data Received**: Check firewall settings, ensure port 8888 is open
- **Parse Errors**: Verify ESP32 is sending correct data format
- **Network Issues**: Ensure both devices are on same network

### Debug Commands
```bash
# Check if UDP port is listening
netstat -uln | grep 8888

# Monitor network traffic
sudo tcpdump -i any udp port 8888

# Check ROS2 topic
ros2 topic info /sensor_raw
```

## Next Steps (Phase 2)
- Multi-device support
- Device registration system
- Variable sensor count handling
- Health monitoring
- Error recovery

## Files Created
- `esp32_sensor_sender.ino`: ESP32 firmware
- `gentact_ros_tools_hybrid/udp_sensor_publisher.py`: ROS2 publisher node
- `launch/udp_sensor_publisher.launch.py`: Launch file
- `UDP_SENSOR_SETUP.md`: This setup guide 