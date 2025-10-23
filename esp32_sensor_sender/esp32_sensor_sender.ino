/*
 * ESP32 Capacitive Sensor UDP Sender
 * 
 * Make sure to create credentials.hpp with your WiFi credentials:
 * - WIFI_SSID
 * - WIFI_PASSWORD  
 * - UDP_ADDRESS
 * - UDP_PORT
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <CapacitiveSensorR4.h>
#include "credentials.hpp"

// WiFi configuration (from credentials.hpp)
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// UDP configuration (from credentials.hpp)
const char* udpAddress = UDP_ADDRESS;
const int udpPort = UDP_PORT;

// USER CONFIGURATION - Define your sensors here
// Format: {send_pin, receive_pin}
const int SENSOR_PINS[][2] = {
    {3, 2},   // Sensor 0
    {3, 4},   // Sensor 1
    {3, 5},   // Sensor 2
    {18, 19}, // Sensor 3
    {18, 17}, // Sensor 4
    {18, 16}, // Sensor 5
    // Add more sensors as needed: {send_pin, receive_pin}
    // {3, 6},   // Sensor 3
    // {3, 7},   // Sensor 4
    // {3, 8},   // Sensor 5
};

// DEVICE CONFIGURATION - Set unique ID for each device
// Change this for each ESP32 device (e.g., 0x00000001, 0x00000002, etc.)
const uint32_t DEVICE_ID = 0x00000001;  // Unique device identifier

// Automatically calculate number of sensors from array
const int NUM_SENSORS = sizeof(SENSOR_PINS) / sizeof(SENSOR_PINS[0]);

// Dynamic sensor array
CapacitiveSensor* sensors[NUM_SENSORS];

// WiFi and UDP objects
WiFiUDP udp;

// Sensor data structure (adapts to number of sensors)
struct SensorData {
  uint32_t device_id;      // 4 bytes
  uint32_t num_sensors;    // 4 bytes
  float sensor_values[NUM_SENSORS];  // Dynamic array
} sensorData;

// Timing variables for 10Hz loop
const unsigned long LOOP_PERIOD = 0;  // 100ms = 10Hz
unsigned long lastLoopTime = 0;

void setup() {
    Serial.begin(115200);
    
    // Initialize sensor data structure
    sensorData.device_id = DEVICE_ID;  // Use the defined DEVICE_ID
    sensorData.num_sensors = NUM_SENSORS;  // Use calculated number of sensors
    
    Serial.print("Initializing ");
    Serial.print(NUM_SENSORS);
    Serial.println(" sensors...");
    
    // Initialize capacitive sensors dynamically
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensors[i] = new CapacitiveSensor(SENSOR_PINS[i][0], SENSOR_PINS[i][1]);
        // sensors[i]->set_CS_AutocaL_Millis(0xFFFFFFFF);  // turn off autocalibrate
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": pins ");
        Serial.print(SENSOR_PINS[i][0]);
        Serial.print(", ");
        Serial.println(SENSOR_PINS[i][1]);
    }
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // Initialize UDP
    udp.begin(udpPort);
    Serial.println("UDP initialized");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Check if it's time for the next loop cycle (10Hz)
    if (currentTime - lastLoopTime >= LOOP_PERIOD) {
        // Read capacitive sensor values
        long start = millis();
        
        // Read all sensors dynamically
        for (int i = 0; i < NUM_SENSORS; i++) {
            long sensorValue = sensors[i]->capacitiveSensor(30);
            sensorData.sensor_values[i] = (float) sensorValue;

            print(sensorValue);
            if (i != NUM_SENSORS - 1) {
                print(",");
            }
        }
        
        // Send data via UDP
        udp.beginPacket(udpAddress, udpPort);
        udp.write((uint8_t*)&sensorData, sizeof(sensorData));
        udp.endPacket();
        
        // Print debug info
        // Serial.print("Performance: ");
        // Serial.print(millis() - start);
        // Serial.print("ms | Loop time: ");
        // Serial.print(currentTime - lastLoopTime);
        // Serial.print("ms | Sent: ");
        // for (int i = 0; i < NUM_SENSORS; i++) {
        //     Serial.print(sensorData.sensor_values[i]);
        //     if (i < NUM_SENSORS - 1) Serial.print(" ");
        // }
        // Serial.println();
        
        // Update last loop time
        lastLoopTime = currentTime;
    }
    
    // Small delay to prevent overwhelming the CPU
    delay(1);
} 