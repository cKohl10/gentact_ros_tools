import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Bool
import numpy as np
import time


class SensorTrackingPublisher(Node):
    def __init__(self):
        super().__init__("sensor_tracking_publisher")

        self.tracking_status = False
        self.baseline_tracking_status = False
        
        # Declare parameters
        self.declare_parameter('num_sensors', 6)
        self.declare_parameter('Kp', 0.15)  # Proportional gain for main tracker
        self.declare_parameter('Kd', 0.001)  # Derivative gain for main tracker
        self.declare_parameter('baseline_kp', 0.01)  # Proportional gain for baseline PID
        self.declare_parameter('baseline_ki', 0.0000001)  # Integral gain for baseline PID
        self.declare_parameter('baseline_kd', 0.000001)  # Derivative gain for baseline PID
        self.declare_parameter('baseline_duration', 3.0)  # Baseline collection duration in seconds
        self.declare_parameter('baseline_timeout', 10.0)  # Max time to wait for baseline collection
        self.declare_parameter('skin_name', '')
        self.declare_parameter('masking_threshold', 200.0)  # Threshold for masking baseline updates

        # Get parameters
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.baseline_kp = self.get_parameter('baseline_kp').get_parameter_value().double_value
        self.baseline_ki = self.get_parameter('baseline_ki').get_parameter_value().double_value
        self.baseline_kd = self.get_parameter('baseline_kd').get_parameter_value().double_value
        self.baseline_duration = self.get_parameter('baseline_duration').get_parameter_value().double_value
        self.baseline_timeout = self.get_parameter('baseline_timeout').get_parameter_value().double_value
        self.skin_name = self.get_parameter('skin_name').get_parameter_value().string_value
        self.masking_threshold = self.get_parameter('masking_threshold').get_parameter_value().double_value
        # Initialize baseline collection
        self.start_time = time.time()
        self.baseline_collected = False
        self.baseline_values = np.zeros(self.num_sensors)
        self.baseline_samples = []
        self.sensor_data_received = False
        self.sensor_values = np.zeros(self.num_sensors)
        
        # Initialize tracker state
        self.X = np.zeros(self.num_sensors)  # Current tracker state
        self.last_X = np.zeros(self.num_sensors)  # Previous tracker state for diff
        
        # Separate timing and error state for tracker and baseline PID
        self.tracker_last_err = np.zeros(self.num_sensors)
        self.tracker_last_time = time.time()
        self.baseline_last_err = np.zeros(self.num_sensors)
        self.baseline_integral_err = np.zeros(self.num_sensors)
        self.baseline_last_time = time.time()
        
        # Track how long each sensor has been masked from baseline updates
        self.baseline_mask_start_times = np.full(self.num_sensors, float('inf'))
        self.baseline_mask_timeout = 15.0  # seconds

        self.tracking_status_sub = self.create_subscription(
            Bool,
            '/sensor_tracking_status',
            self.tracking_status_callback,
            1
        )

        self.baseline_tracking_status_sub = self.create_subscription(
            Bool,
            '/sensor_baseline_tracking_status',
            self.baseline_tracking_status_callback,
            1
        )
        
        # Subscribe to raw sensor data
        if self.skin_name == '':
            self.subscription = self.create_subscription(
                Int32MultiArray,
                '/sensor_raw',
                self.sensor_callback,
                1
            )
            # Create publishers
            self.tracking_publisher = self.create_publisher(Int32MultiArray, '/sensor_tracking', 1)
            self.baseline_publisher = self.create_publisher(Int32MultiArray, '/sensor_baseline', 1)
        else:
            self.subscription = self.create_subscription(
                Int32MultiArray,
                f'/sensor_raw_{self.skin_name}',
                self.sensor_callback,
                1
            )
            # Create publishers
            self.tracking_publisher = self.create_publisher(Int32MultiArray, f'/sensor_tracking_{self.skin_name}', 1)
            self.baseline_publisher = self.create_publisher(Int32MultiArray, f'/sensor_baseline_{self.skin_name}', 1)
        
        # Create timer to check for sensor data timeout
        self.timeout_timer = self.create_timer(1.0, self.check_sensor_timeout)
        
        self.get_logger().info(f"Sensor tracking publisher initialized for {self.num_sensors} sensors")
        self.get_logger().info(f"Main tracker PD gains: Kp={self.Kp}, Kd={self.Kd}")
        self.get_logger().info(f"Baseline PID gains: Kp={self.baseline_kp}, Ki={self.baseline_ki}, Kd={self.baseline_kd}")
        self.get_logger().info(f"Waiting for sensor data on /sensor_raw...")
        self.get_logger().info(f"Will collect baseline for {self.baseline_duration}s (timeout: {self.baseline_timeout}s)")

    
    def tracking_status_callback(self, msg: Bool):
        self.tracking_status = msg.data
    
    def baseline_tracking_status_callback(self, msg: Bool):
        self.baseline_tracking_status = msg.data
        # Set the baseline to the most recent sensor values
        print("====== Activated Baseline Tracking ======")
        print(f"Setting baseline to {self.sensor_values}")
        self.baseline_values = self.sensor_values.copy() - self.baseline_values
        self.baseline_collected = True
        print("====== Baseline Collected ======")
    
    def check_sensor_timeout(self):
        """Check if we're stuck waiting for sensor data"""
        elapsed = time.time() - self.start_time
        
        if not self.sensor_data_received and elapsed > 5.0:
            self.get_logger().warn(f"No sensor data received after {elapsed:.1f}s. Check if /sensor_raw topic is publishing.")
            
        if not self.baseline_collected and elapsed > self.baseline_timeout:
            self.get_logger().error(f"Baseline collection timeout after {elapsed:.1f}s. Using zero baseline.")
            self.baseline_collected = True
            self.baseline_values = np.zeros(self.num_sensors)

    def collect_baseline(self, sensor_values):
        """Collect baseline values for initial calibration"""
        if self.baseline_collected:
            return True
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if elapsed_time < self.baseline_duration:
            # Still collecting baseline
            self.baseline_samples.append(sensor_values.copy())
            
            # Log progress every 30 samples
            if len(self.baseline_samples) % 30 == 0:
                remaining_time = self.baseline_duration - elapsed_time
                self.get_logger().info(f"Collecting baseline... {remaining_time:.1f}s remaining ({len(self.baseline_samples)} samples)")
            
            return False
        else:
            # Baseline collection complete
            if self.baseline_samples:
                self.baseline_values = np.mean(self.baseline_samples, axis=0)
                self.get_logger().info(f"Baseline collection complete! {len(self.baseline_samples)} samples")
                self.get_logger().info(f"Baseline values: {self.baseline_values}")
            else:
                self.get_logger().warn("No samples collected for baseline. Using zeros.")
                
            self.baseline_collected = True
            return True

    def update_tracker(self, signal):
        """Update main PD tracker to follow the signal"""
        current_time = time.time()
        dt = max(current_time - self.tracker_last_time, 1e-6)
        
        # Calculate tracking error and derivative
        err = signal - self.X
        derr = (err - self.tracker_last_err) / dt
        
        # Update tracker state using PD law
        self.X += self.Kp * err + self.Kd * derr
        
        # Update state for next iteration
        self.tracker_last_err = err.copy()
        self.tracker_last_time = current_time
        
        return self.X.copy()
    
    def update_baseline(self, signal):

        """Update baseline values slowly using PID control"""
        current_time = time.time()
        dt = max(current_time - self.baseline_last_time, 1e-6)
        
        # Calculate PID terms
        err = signal - self.baseline_values

        # Don't track to super strong signals (element-wise)
        strong_signal_mask = np.abs(err) > self.masking_threshold
        
        # Track mask timing for each sensor
        for i in range(self.num_sensors):
            if strong_signal_mask[i]:
                # Sensor has strong signal
                if self.baseline_mask_start_times[i] == float('inf'):
                    # First time being masked, record start time
                    self.baseline_mask_start_times[i] = current_time
                elif current_time - self.baseline_mask_start_times[i] > self.baseline_mask_timeout:
                    # Been masked too long, force baseline update
                    strong_signal_mask[i] = False
            else:
                # No strong signal, reset mask timer
                self.baseline_mask_start_times[i] = float('inf')
        
        derr = (err - self.baseline_last_err) / dt
        self.baseline_integral_err += err * dt
        
        # Apply PID control
        pid_output = (self.baseline_kp * err + 
                     self.baseline_ki * self.baseline_integral_err + 
                     self.baseline_kd * derr)
        
        # Only update baseline for sensors without strong signals (or timeout exceeded)
        pid_output[strong_signal_mask] = 0
        self.baseline_values += pid_output
        
        # Update state for next iteration
        self.baseline_last_err = err.copy()
        self.baseline_last_time = current_time
        
        return self.baseline_values.copy()

    def get_diff(self):
        """Calculate difference between current and previous tracker state"""
        diff = np.abs(self.X - self.last_X)
        self.last_X = self.X.copy()
        return diff

    def sensor_callback(self, msg):
        """Process incoming sensor data"""

        self.sensor_data_received = True
                
        # Convert and validate sensor data
        sensor_values = np.array(msg.data, dtype=float)
        if len(sensor_values) != self.num_sensors:
            # Resize array to match expected number of sensors
            if len(sensor_values) > self.num_sensors:
                sensor_values = sensor_values[:self.num_sensors]
            else:
                sensor_values = np.pad(sensor_values, (0, self.num_sensors - len(sensor_values)))

        if self.tracking_status and self.baseline_tracking_status:
            try:
                # Check if baseline collection is complete
                if not self.collect_baseline(sensor_values):
                    return  # Still collecting baseline
                
                # Update baseline slowly (tracks long-term drift)
                self.update_baseline(sensor_values)
                
                # Calculate calibrated values and update tracker
                calibrated_values = sensor_values - self.baseline_values
                tracked_values = self.update_tracker(calibrated_values)
                diff_values = self.get_diff()
                
                # Publish data (convert to integers)
                tracking_data = tracked_values
                self.tracking_publisher.publish(Int32MultiArray(
                    data=np.round(tracking_data).astype(int).tolist()
                ))
                
                self.baseline_publisher.publish(Int32MultiArray(
                    data=np.round(self.baseline_values).astype(int).tolist()
                ))
                
                # Debug logging
                self.get_logger().debug(f"Raw: {sensor_values}")
                self.get_logger().debug(f"Baseline: {self.baseline_values}")
                self.get_logger().debug(f"Tracked: {tracked_values}")
                
            except Exception as e:
                self.get_logger().error(f"Error in sensor_callback: {e}")

        elif not self.tracking_status and self.baseline_tracking_status:
            self.sensor_data_received = True
            try:
                # Check if baseline collection is complete
                if not self.collect_baseline(sensor_values):
                    return  # Still collecting baseline
                
                # Update baseline slowly (tracks long-term drift)
                self.update_baseline(sensor_values)

                calibrated_values = sensor_values - self.baseline_values

                self.tracking_publisher.publish(Int32MultiArray(
                    data=np.round(calibrated_values).astype(int).tolist()
                ))

                self.baseline_publisher.publish(Int32MultiArray(
                    data=np.round(self.baseline_values).astype(int).tolist()
                ))
            except Exception as e:
                self.get_logger().error(f"Error in sensor_callback: {e}")
        else:
            self.tracking_publisher.publish(Int32MultiArray(
                    data=np.round(sensor_values).astype(int).tolist()
                ))
            self.sensor_values = sensor_values.copy()



def main(args=None):
    rclpy.init(args=args)
    
    try:
        sensor_tracking_pub = SensorTrackingPublisher()
        rclpy.spin(sensor_tracking_pub)
    except KeyboardInterrupt:
        pass
    finally:
        if 'sensor_tracking_pub' in locals():
            sensor_tracking_pub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()