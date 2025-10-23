# Credentials Setup

This project uses a gitignored `credentials.hpp` file to store sensitive WiFi credentials for the ESP32 sensor.

## Setup Instructions

### 1. Create credentials.hpp

Create a `credentials.hpp` file in the `esp32_sensor_sender/` folder with your WiFi credentials:

```cpp
#ifndef CREDENTIALS_HPP
#define CREDENTIALS_HPP

// WiFi Configuration
const char* WIFI_SSID = "your_wifi_network_name";
const char* WIFI_PASSWORD = "your_wifi_password";

// UDP Configuration  
const char* UDP_ADDRESS = "192.168.1.100";  // Your ROS2 machine IP
const int UDP_PORT = 8888;

#endif // CREDENTIALS_HPP
```

### 2. Update the credentials

Replace the placeholder values in `esp32_sensor_sender/credentials.hpp`:
- `WIFI_SSID`: Your WiFi network name
- `WIFI_PASSWORD`: Your WiFi password
- `UDP_ADDRESS`: The IP address of your ROS2 machine
- `UDP_PORT`: UDP port (default 8888)

### 3. Upload to ESP32

The ESP32 code will automatically include the credentials from `credentials.hpp`. Simply upload the `esp32_sensor_sender.ino` file to your ESP32.

## Security Notes

- The `credentials.hpp` file is already added to `.gitignore` to prevent accidentally committing credentials
- Never commit the actual credentials file to version control
- Keep a backup of your credentials in a secure location
- Consider using environment variables for production deployments

## Troubleshooting

If you get compilation errors:
1. Make sure `credentials.hpp` exists in the `esp32_sensor_sender/` folder
2. Check that the WiFi credentials are correct
3. Verify the UDP address is reachable from the ESP32's network 