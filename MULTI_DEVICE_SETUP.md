# Multi-Device Setup Guide

This guide explains how to set up multiple ESP32 devices to connect to the ROS node simultaneously.

## Overview

The system now supports multiple ESP32 devices, each with a unique device ID. Each device publishes to its own ROS topic, allowing for independent sensor data streams.

## Step-by-Step Setup

### 1. Configure Each ESP32 Device

For each ESP32 device, you need to:

1. **Set a unique device ID** in `esp32_sensor_sender.ino`:
   ```cpp
   // Change this for each ESP32 device
   const uint32_t DEVICE_ID = 0x00000001;  // Device 1
   const uint32_t DEVICE_ID = 0x00000002;  // Device 2
   const uint32_t DEVICE_ID = 0x00000003;  // Device 3
   // etc.
   ```

2. **Configure WiFi credentials** in `credentials.hpp`:
   ```cpp
   #define WIFI_SSID "YourWiFiNetwork"
   #define WIFI_PASSWORD "YourWiFiPassword"
   #define UDP_ADDRESS "192.168.1.100"  // ROS computer IP
   #define UDP_PORT 8888
   ```

3. **Configure sensor pins** for each device as needed:
   ```cpp
   const int SENSOR_PINS[][2] = {
       {3, 2},   // Sensor 0
       {3, 4},   // Sensor 1
       // Add more sensors as needed
   };
   ```

### 2. Upload Code to Each Device

1. Connect each ESP32 to your computer
2. Upload the modified `esp32_sensor_sender.ino` to each device
3. Verify each device connects to WiFi and starts sending data

### 3. Start the ROS Node

Launch the UDP sensor publisher:
```bash
ros2 launch gentact_ros_tools_hybrid udp_sensor_publisher.launch.py
```

## ROS Topics

Each device will publish to its own topic:
- Device 1 (0x00000001): `/sensor_raw/device_00000001`
- Device 2 (0x00000002): `/sensor_raw/device_00000002`
- Device 3 (0x00000003): `/sensor_raw/device_00000003`
- etc.

## Monitoring

### Check Active Devices
```bash
ros2 topic list | grep sensor_raw
```

### Monitor Specific Device
```bash
ros2 topic echo /sensor_raw/device_00000001
```

### View All Sensor Data
```bash
ros2 topic echo /sensor_raw/device_00000001 --once
ros2 topic echo /sensor_raw/device_00000002 --once
```

## Configuration Parameters

The ROS node supports these parameters:
- `udp_port`: UDP port to listen on (default: 8888)
- `buffer_size`: UDP buffer size (default: 1024)
- `timeout_seconds`: Timeout for device inactivity (default: 5.0)
- `max_devices`: Maximum number of devices to support (default: 10)

## Troubleshooting

### Device Not Connecting
1. Check WiFi credentials in `credentials.hpp`
2. Verify UDP_ADDRESS points to the ROS computer
3. Check firewall settings on the ROS computer

### Device ID Conflicts
- Ensure each ESP32 has a unique `DEVICE_ID`
- Device IDs should be in range 0x00000001 to 0xFFFFFFFF

### ROS Node Issues
1. Check if UDP port 8888 is available
2. Verify network connectivity between ESP32 and ROS computer
3. Check ROS node logs for error messages

## Example Device Configurations

### Device 1 (Robot Arm Sensors)
```cpp
const uint32_t DEVICE_ID = 0x00000001;
const int SENSOR_PINS[][2] = {
    {3, 2},   // Link 1 sensor
    {3, 4},   // Link 2 sensor
    {3, 5},   // Link 3 sensor
};
```

### Device 2 (End Effector Sensors)
```cpp
const uint32_t DEVICE_ID = 0x00000002;
const int SENSOR_PINS[][2] = {
    {18, 19}, // Finger 1
    {18, 17}, // Finger 2
    {18, 16}, // Finger 3
};
```

### Device 3 (Environmental Sensors)
```cpp
const uint32_t DEVICE_ID = 0x00000003;
const int SENSOR_PINS[][2] = {
    {25, 26}, // Proximity sensor
    {27, 28}, // Contact sensor
};
```

## Performance Considerations

- Each device sends data at 10Hz
- UDP packets are ~40 bytes per device
- Maximum 10 devices supported by default
- Consider network bandwidth for large deployments 