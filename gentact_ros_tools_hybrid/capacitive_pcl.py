import math
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32MultiArray, Float64, Float64MultiArray, Bool
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
from typing import List
import os
from ament_index_python.packages import get_package_share_directory
import json
from tf2_ros import TransformListener, Buffer

class CapacitivePCL(Node):
    def __init__(self):
        super().__init__("capacitive_pcl")

        # Declare parameters
        self.declare_parameter('num_sensors', 6)
        self.declare_parameter('max_distance', 0.1)  # Maximum distance threshold in meters (10 cm)
        self.declare_parameter('frame_id', 'link5')
        self.declare_parameter('output_frame', 'map')
        self.declare_parameter('time_offset_sec', 0.1)
        self.declare_parameter('skin_name', '')
        self.declare_parameter('multiplier', None)  # Multiplier for each sensor

        # Get parameters
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        self.max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.time_offset_sec = self.get_parameter('time_offset_sec').get_parameter_value().double_value
        self.base_points_per_sensor = 1
        self.skin_name = self.get_parameter('skin_name').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.multiplier = self.get_parameter('multiplier').get_parameter_value().double_array_value
        if self.multiplier is None or len(self.multiplier) == 0 or len(self.multiplier) != self.num_sensors:
            self.get_logger().info(f"No valid multiplier provided, defaulting to array of ones for {self.num_sensors} sensors")
            self.multiplier = np.ones(self.num_sensors)

        # NOTE: To prevent overflow, we prematurely apply at 10^13 magnatude shift due to approximate the permittivity of air
        self.clock_freq = 160.0  # 160 MHz clock frequency for Qwiic board
        self.R = [10.0] * self.num_sensors  # Resistance values for each sensor
        self.alpha = 0.0005  # Initial tuning parameter

        self.envelopes = [1.0] * self.num_sensors
        self.baseline = [0] * self.num_sensors  # Initialize baseline to zeros
        
        # Track whether we've received tracking data yet
        self.using_tracked_data = False

        if self.skin_name == '':

            self.tuning_sub = self.create_subscription(
                Float64,
                '/sensor_tuning',
                self.tuning_callback,
                1
            )
            self.tuning_sub  # prevent unused variable warning

            self.tracking_sub = self.create_subscription(
                Int32MultiArray,
                '/sensor_tracking',
                self.tracking_callback,
                1
            )

            self.raw_sub = self.create_subscription(
                Int32MultiArray,
                '/sensor_raw',
                self.raw_callback,
                1
            )
            self.raw_sub  # prevent unused variable warning

            # Create a combined pointcloud publisher in the chosen output frame
            self.combined_pcl_pub = self.create_publisher(
                PointCloud2,
                '/sensors_pcl',
                10
            )

            self.dist_pub = self.create_publisher(
                Float64MultiArray,
                '/sensor_dist',
                1
            )

            self.obstacle_pub = self.create_publisher(
                PoseStamped,
                '/obstacle',
                1
            )

            self.baseline_sub = self.create_subscription(
                Int32MultiArray,
                '/sensor_baseline',
                self.baseline_callback,
                1
            )

            self.status_pub = self.create_publisher(
                Bool,
                '/sensor_status',
                1
            )

            self.tracking_status_sub = self.create_subscription(
                Bool,
                '/sensor_tracking_status',
                self.tracking_status_callback,
                1
            )
        else:
            self.tuning_sub = self.create_subscription(
                Float64,
                f'/sensor_tuning_{self.skin_name}',
                self.tuning_callback,
                1
            )
            self.tuning_sub  # prevent unused variable warning

            self.tracking_sub = self.create_subscription(
                Int32MultiArray,
                f'/sensor_tracking_{self.skin_name}',
                self.tracking_callback,
                1
            )

            self.raw_sub = self.create_subscription(
                Int32MultiArray,
                f'/sensor_raw_{self.skin_name}',
                self.raw_callback,
                1
            )
            self.raw_sub  # prevent unused variable warning

            # Create a combined pointcloud publisher in the chosen output frame
            self.combined_pcl_pub = self.create_publisher(
                PointCloud2,
                f'/sensors_pcl_{self.skin_name}',
                10
            )

            self.dist_pub = self.create_publisher(
                Float64MultiArray,
                f'/sensor_dist_{self.skin_name}',
                1
            )

            self.obstacle_pub = self.create_publisher(
                PoseStamped,
                f'/obstacle_{self.skin_name}',
                1
            )

            # self.obstacle_pub = self.create_publisher(
            #     PoseStamped,
            #     f'/obstacle',
            #     1
            # )

            self.baseline_sub = self.create_subscription(
                Int32MultiArray,
                f'/sensor_baseline_{self.skin_name}',
                self.baseline_callback,
                1
            )

            self.status_pub = self.create_publisher(
                Bool,
                f'/sensor_status_{self.skin_name}',
                1
            )

            self.tracking_status_sub = self.create_subscription(
                Bool,
                f'/sensor_tracking_status_{self.skin_name}',
                self.tracking_status_callback,
                1
            )

        
        # Initialize base pointcloud pattern (small grid)
        self.base_points = self.generate_base_points()
        
        self.get_logger().info(f"Capacitive PCL node initialized for {self.num_sensors} sensors")
        self.get_logger().info("Starting with raw sensor data, will switch to tracked data when available")

        # Debug accumulation for per-second stats
        self._debug_values_per_sensor = [[] for _ in range(self.num_sensors)]
        # self._debug_timer = self.create_timer(1.0, self._emit_debug_stats)

        self.status_pub.publish(Bool(data=True))
        self.heartbeat_timer = self.create_timer(3.0, self.heartbeat_publisher)
        self.status = True

    def tracking_status_callback(self, msg: Bool):
        self.using_tracked_data = msg.data

    def heartbeat_publisher(self):
        self.status_pub.publish(Bool(data=self.status))
        self.status = False

    def tuning_callback(self, msg: Float64):
        self.alpha = float(msg.data)
        self.get_logger().info(f"Received tuning alpha: {self.alpha}")
    
    def baseline_callback(self, msg: Int32MultiArray):
        if len(msg.data) >= self.num_sensors:
            self.baseline = msg.data
        else:
            self.get_logger().warn(f"Received baseline with {len(msg.data)} elements, expected {self.num_sensors}. Ignoring update.")
        # self.get_logger().info(f"Received baseline: {self.baseline}")
    
    def tracking_callback(self, msg: Int32MultiArray):
        """Handle tracking data - switch to tracked mode and process"""
        if not self.using_tracked_data:
            return
            # self.using_tracked_data = True
            # self.get_logger().info("Switched to using tracked sensor data")
        
        self.sensor_callback(msg)
    
    def raw_callback(self, msg: Int32MultiArray):
        """Handle raw data - only process if not using tracked data yet"""
        if not self.using_tracked_data:
            self.sensor_callback(msg)
    
    def generate_base_points(self):
        """Generate a base pattern of points for each sensor"""
        # Create a small grid of points in XY plane
        grid_size = int(np.sqrt(self.base_points_per_sensor))
        if grid_size * grid_size < self.base_points_per_sensor:
            grid_size += 1

        x = [0]
        y = [0]  # 1cm spread in Y

        points = []
        count = 0
        for xi in x:
            for yi in y:
                if count < self.base_points_per_sensor:
                    points.append([xi, yi, 0.0])  # Z will be modified by distance
                    count += 1
        
        return np.array(points)
    
    def create_pointcloud_msg(self, point, sensor_id):
        """Create a PointCloud2 message from numpy array of points"""
        # Create header
        msg = PointCloud2()
        # Use a timestamp slightly in the past to ensure TF transforms are available
        current_time = self.get_clock().now()
        # Subtract a small offset to ensure transforms are available
        past_time = current_time - Duration(seconds=self.time_offset_sec)
        msg.header.stamp = past_time.to_msg()
        msg.header.frame_id = f"{self.frame_id}_sensor_{sensor_id}"
        
        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Set up the cloud data
        msg.height = 1
        msg.width = 1
        msg.point_step = 16  # 4 floats * 4 bytes (x, y, z, intensity)
        msg.row_step = msg.point_step * msg.width
        msg.is_bigendian = False
        msg.is_dense = True
        
        # Pack the data
        buffer = []
        buffer.append(struct.pack('ffff', float(point[0]), float(point[1]), float(point[2]), float(point[3])))
        
        msg.data = b''.join(buffer)
        
        return msg

    def create_pointcloud_msg_multi(self, points: np.ndarray, frame_id: str) -> PointCloud2:
        """Create a PointCloud2 message from an Nx3 numpy array of points in the given frame."""
        msg = PointCloud2()
        current_time = self.get_clock().now()
        past_time = current_time - Duration(seconds=self.time_offset_sec)
        msg.header.stamp = past_time.to_msg()
        msg.header.frame_id = frame_id

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        if points.size == 0:
            msg.height = 1
            msg.width = 0
            msg.point_step = 16
            msg.row_step = 0
            msg.is_bigendian = False
            msg.is_dense = True
            msg.data = b''
            return msg

        points = points.astype(np.float32)
        msg.height = 1
        msg.width = points.shape[0]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_bigendian = False
        msg.is_dense = True

        buffer = []
        for p in points:
            buffer.append(struct.pack('ffff', float(p[0]), float(p[1]), float(p[2]), float(p[3])))
        msg.data = b''.join(buffer)
        return msg
    
    def sensor_callback(self, msg):
        """Callback function for sensor data"""
        try:
            # Get sensor values
            sensor_values = msg.data
            
            # Check if we have any sensor data at all
            if len(sensor_values) == 0:
                self.get_logger().warn("Received empty sensor data, skipping processing")
                return
            
            # Ensure we have the expected number of sensors
            num_available = min(len(sensor_values), self.num_sensors)
            
            if len(sensor_values) < self.num_sensors:
                self.get_logger().debug(f"Received {len(sensor_values)} sensor values, expected {self.num_sensors}")
            dist_values = []
            all_sensor_points = []  # in sensor frames
            combined_points_map = []  # transformed to output frame
            raw_dist_values = []
            self.status = True
            
            for i in range(num_available):
                # Get distance value and convert to meters
                # distance = sensor_values[i] * self.distance_scale
                if self.using_tracked_data:
                    capacitance = -(np.abs(sensor_values[i])) / (self.clock_freq * self.R[i] * 30.0 * math.log(0.5))
                else:
                    capacitance = -(np.abs(sensor_values[i] - self.baseline[i])) / (self.clock_freq * self.R[i] * 30.0 * math.log(0.5))
                distance = (self.alpha / np.abs(capacitance)) * self.multiplier[i]
                raw_dist_values.append(distance)
                # Accumulate predicted distance for per-second debug stats
                # if i < len(self._debug_values_per_sensor):
                #     self._debug_values_per_sensor[i].append(float(distance))

                # Create pointcloud for this sensor
                
                
                # Copy base points and offset them by the distance in Z
                sensor_points = np.array([0.0, 0.0, 0.0])
                
                if distance <= self.max_distance and distance >= 0.0:
                    # Within range: use actual distance
                    sensor_points[2] = distance
                    self.get_logger().debug(f"Sensor {i} within range: {distance:.6f}m")
                else:
                    # Out of range: publish at -0.2 * max_distance
                    sensor_points[2] = 10000 * self.max_distance
                    self.get_logger().debug(f"Sensor {i} out of range ({distance:.6f}m), publishing at {-0.2 * self.max_distance:.6f}m")
                    distance = self.max_distance*1000
                
                # Accumulate point in sensor frame for obstacle calc
                all_sensor_points.append(sensor_points)

                # Transform to output frame (e.g., map) and accumulate for combined cloud
                source_frame = f"{self.frame_id}_sensor_{i}"
                try:
                    map_transform = self.tf_buffer.lookup_transform(
                        self.output_frame,
                        source_frame,
                        rclpy.time.Time()
                    )
                    transformed_point = self._transform_point(sensor_points, map_transform)
                    combined_points_map.append(transformed_point)
                except Exception:
                    # Skip if TF not ready for this sensor
                    pass
                dist_values.append(distance)
            dist_msg = Float64MultiArray(data=raw_dist_values)
            self.dist_pub.publish(dist_msg)
            
            # self.get_logger().debug(f"Published pointclouds for {num_available} sensors")

            # Publish combined point cloud in output frame
            if len(combined_points_map) > 0:
                combined_array = np.array(combined_points_map, dtype=np.float32)
                combined_msg = self.create_pointcloud_msg_multi(combined_array, self.output_frame)
                self.combined_pcl_pub.publish(combined_msg)

            # Get lowest distance sensor (only if we have valid distance values)
            if len(raw_dist_values) > 0:
                min_distance_sensor = np.argmin(raw_dist_values)
                min_distance = raw_dist_values[min_distance_sensor]
                # self.get_logger().info(f"Lowest distance sensor: {min_distance_sensor}")

                if min_distance < self.max_distance:
                    msg = self.get_obstacle_pose(min_distance_sensor, all_sensor_points[min_distance_sensor])
                    self.obstacle_pub.publish(msg)
                else:
                    zero_pose = PoseStamped()
                    zero_pose.header.stamp = self.get_clock().now().to_msg()
                    zero_pose.header.frame_id = 'map'
                    zero_pose.pose.position.x = 100.0
                    zero_pose.pose.position.y = 0.0
                    zero_pose.pose.position.z = 0.0
                    self.obstacle_pub.publish(zero_pose)
            else:
                # No sensor data processed, publish default "no obstacle" pose
                zero_pose = PoseStamped()
                zero_pose.header.stamp = self.get_clock().now().to_msg()
                zero_pose.header.frame_id = 'map'
                zero_pose.pose.position.x = 100.0
                zero_pose.pose.position.y = 0.0
                zero_pose.pose.position.z = 0.0
                self.obstacle_pub.publish(zero_pose)

        except Exception as e:
            self.get_logger().error(f"Error in sensor_callback: {e}")

    def _emit_debug_stats(self):
        """Emit per-second debug message with per-sensor mean and std of predicted distances."""
        try:
            means = []
            stds = []
            counts = []
            for i in range(self.num_sensors):
                values = self._debug_values_per_sensor[i] if i < len(self._debug_values_per_sensor) else []
                if len(values) > 0:
                    means.append(float(np.mean(values)))
                    stds.append(float(np.std(values)))
                    counts.append(len(values))
                else:
                    means.append(float('nan'))
                    stds.append(float('nan'))
                    counts.append(0)

            # Round for readability
            means_rounded = np.round(means, 4).tolist()
            stds_rounded = np.round(stds, 4).tolist()

            self.get_logger().info(
                f"Predicted distances mean per sensor [m]: {means_rounded} | std [m]: {stds_rounded} | samples: {counts}"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to compute debug stats: {e}")
        finally:
            # Reset buffers for the next second
            self._debug_values_per_sensor = [[] for _ in range(self.num_sensors)]

    def get_obstacle_pose(self, sensor_id, sensor_point):
        """Get the pose of the obstacle"""
        map_transform = self.tf_buffer.lookup_transform(
            "map",
            f"{self.frame_id}_sensor_{sensor_id}",
            rclpy.time.Time()
        )

        transformed_point = self._transform_point(sensor_point, map_transform)

        """Create an obstacle message from a single 3D point."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Always publish in 'map' frame

        # Use the already transformed point
        msg.pose.position.x = float(transformed_point[0])
        msg.pose.position.y = float(transformed_point[1])
        msg.pose.position.z = float(transformed_point[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        return msg

    def _transform_point(self, point, transform):
            """Transform a point from sensor frame to map frame using rotation and translation."""
            # Extract quaternion and translation from transform
            q = transform.transform.rotation
            t = transform.transform.translation
            
            # Convert quaternion to rotation matrix manually
            # Quaternion components: [w, x, y, z]
            w, x, y, z = q.w, q.x, q.y, q.z
            
            # Normalize quaternion
            norm = np.sqrt(w*w + x*x + y*y + z*z)
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
            
            # Rotation matrix from quaternion
            rotation_matrix = np.array([
                [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y],
                [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x],
                [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
            ])
            
            # Apply rotation first, then translation
            rotated_point = rotation_matrix @ point
            transformed_point = rotated_point + np.array([t.x, t.y, t.z])
            transformed_point = np.append(transformed_point, point[2])
            
            return transformed_point

    def update_envelopes(self, values):
        for i in range(self.num_sensors):
            if values[i] > self.envelopes[i]:
                self.envelopes[i] = values[i]
            else:
                self.envelopes[i] = self.envelopes[i] * 0.99

def main(args=None):
    rclpy.init(args=args)
    
    capacitive_pcl = CapacitivePCL()
    
    try:
        rclpy.spin(capacitive_pcl)
    except KeyboardInterrupt:
        pass
    finally:
        capacitive_pcl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
