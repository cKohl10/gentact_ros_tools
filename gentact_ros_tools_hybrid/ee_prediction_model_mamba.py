#!/home/carson/ros2_ws/venv/bin/python3
# This is a machine learning model that predicts the end effector position of a robot arm based on the sensor data.
import rclpy
from rclpy.node import Node
import numpy as np
import os
import torch
import torch.nn as nn
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import struct
from sklearn.preprocessing import StandardScaler
from mamba_ssm import Mamba
import tf2_ros
from tf2_ros import TransformException
import geometry_msgs.msg

class MambaModel(nn.Module):
    def __init__(self, input_size=6, hidden_size=64, output_size=3, 
                 d_state=16, d_conv=4, expand=2, num_layers=2):
        super(MambaModel, self).__init__()
        
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        
        # Input projection
        self.input_proj = nn.Linear(input_size, hidden_size)
        
        self.mamba_layers = nn.ModuleList([
            Mamba(
                d_model=hidden_size,
                d_state=d_state,
                d_conv=d_conv,
                expand=expand
            ) for _ in range(num_layers)
        ])
        
        # Layer normalization for each Mamba layer
        self.norms = nn.ModuleList([
            nn.LayerNorm(hidden_size) for _ in range(num_layers)
        ])
        
        # Output projection
        self.output_proj = nn.Linear(hidden_size, output_size)
        self.dropout = nn.Dropout(0.1)
        
    def forward(self, x):
        # x shape: (batch_size, input_size)
        x = x.unsqueeze(1)
        
        # Project to hidden dimension
        x = self.input_proj(x)  # (batch_size, 1, hidden_size)
        
        for i, (mamba, norm) in enumerate(zip(self.mamba_layers, self.norms)):
            residual = x
            x = mamba(x)
            x = norm(x)
            x = residual + x
            x = self.dropout(x)
        
        x = x.squeeze(1)  # (batch_size, hidden_size)
        
        # Project to output
        x = self.output_proj(x)  # (batch_size, output_size)
        
        return x 

class EEPredictionModelNode(Node):
    def __init__(self):
        super().__init__('ee_prediction_model')

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
            print("GPU is available and will be used.")
        else:
            self.device = torch.device("cpu")
            print("GPU is not available, using CPU.")
        
        # Initialize default model parameters
        self.hidden_size = 64
        self.output_size = 3
        self.d_state = 16
        self.d_conv = 4
        self.expand = 2
        self.num_layers = 2
        self.input_size = 6  # Default input size

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Load model from the trained .pth file
        # Use ROS2 parameter system with proper package path
        default_model_path = os.path.join(get_package_share_directory('gentact_ros_tools_hybrid'), 'config', 'mlp_model.pth')
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('sensor_frame_id', 'calibration_skin')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.sensor_frame_id = self.get_parameter('sensor_frame_id').get_parameter_value().string_value
        
        # Load the trained model and configuration
        if os.path.exists(model_path):
            try:
                # Add safe globals for sklearn scalers
                torch.serialization.add_safe_globals([StandardScaler])
                
                # Load with weights_only=False for backward compatibility
                checkpoint = torch.load(model_path, map_location=self.device, weights_only=False)
                
                # Load model configuration from checkpoint
                self.scaler_X = checkpoint.get('scaler_X')
                self.scaler_y = checkpoint.get('scaler_y')
                self.input_cols = checkpoint.get('input_cols', [f'sensor_{i}_cc' for i in range(6)])
                self.output_cols = checkpoint.get('output_cols', ['ee_x', 'ee_y', 'ee_z'])
                
                # Determine input size from scaler
                if self.scaler_X is not None:
                    self.input_size = self.scaler_X.n_features_in_
                
                # Initialize model with correct architecture
                self.model = MambaModel(
                    input_size=self.input_size,
                    hidden_size=self.hidden_size,
                    output_size=self.output_size,
                    d_state=self.d_state,
                    d_conv=self.d_conv,
                    expand=self.expand,
                    num_layers=self.num_layers
                )

                self.model.to(self.device)
                
                # Load trained weights
                self.model.load_state_dict(checkpoint['model_state_dict'])
                self.model.eval()
                
                self.get_logger().info(f'Model loaded successfully from {model_path}')
                self.get_logger().info(f'Input size: {self.input_size}, Output size: {self.output_size}')
                self.get_logger().info(f'Input columns: {self.input_cols}')
                self.get_logger().info(f'Output columns: {self.output_cols}')
                
            except Exception as e:
                self.get_logger().error(f'Error loading model: {e}')
                raise RuntimeError(f'Failed to load model from {model_path}: {e}')
        else:
            self.get_logger().error(f'Model file {model_path} not found.')
            raise FileNotFoundError(f'Model file not found: {model_path}')

        self.sensor_sub = self.create_subscription(
            Int32MultiArray,
            '/sensor_raw',
            self.prediction_callback,
            1
        )
        self.prediction_pub = self.create_publisher(
            PointCloud2,
            '/ee_mamba_prediction',
            1
        )
        self.avoidance_obstacle_pub = self.create_publisher(
            PoseStamped,
            '/obstacle',
            1
        )
        self.get_logger().info('EE Prediction Model Node Initialized')
    
    def prediction_callback(self, msg: Int32MultiArray):
        sensor_data = np.array(msg.data)
        
        # Ensure we have the right number of sensor inputs
        if len(sensor_data) != self.input_size:
            self.get_logger().warn(f'Expected {self.input_size} sensor inputs, got {len(sensor_data)}')
            return
        
        # Apply the same scaler used during training
        if self.scaler_X is not None:
            try:
                sensor_data = self.scaler_X.transform(sensor_data.reshape(1, -1)).flatten()
            except Exception as e:
                self.get_logger().error(f'Error applying scaler: {e}')
                return
        else:
            # Fallback to simple normalization if no scaler is available
            sensor_data = (sensor_data - sensor_data.mean()) / (sensor_data.std() + 1e-8)
        
        # Move tensor to the same device as the model
        sensor_data = torch.tensor(sensor_data, dtype=torch.float32).unsqueeze(0).to(self.device)

        # Transform the point from sensor frame to map frame
        map_transform = self.tf_buffer.lookup_transform(
            'map',
            self.sensor_frame_id,
            rclpy.time.Time()
        )
        
        # Predict the end effector position
        with torch.no_grad():
            try:
                prediction = self.model(sensor_data)
                
                # Apply inverse scaling to get actual coordinates
                if self.scaler_y is not None:
                    prediction_np = prediction.cpu().numpy()
                    prediction_unscaled = self.scaler_y.inverse_transform(prediction_np)
                    prediction = torch.tensor(prediction_unscaled, dtype=torch.float32).to(self.device)
                
            except Exception as e:
                self.get_logger().error(f'Error making prediction: {e}')
                return
        
        # Transform the predicted point from sensor frame to map frame
        predicted_point = prediction[0].cpu().numpy()
        transformed_point = self._transform_point(predicted_point, map_transform)
        
        # Convert the prediction to a PointCloud2 message
        point_cloud_msg = self._create_pointcloud2_message(transformed_point)
        obstacle_msg = self._create_obstacle_message(transformed_point)
        self.prediction_pub.publish(point_cloud_msg)
        self.avoidance_obstacle_pub.publish(obstacle_msg)
        self.get_logger().debug(f'Published prediction: x={prediction[0, 0]:.3f}, y={prediction[0, 1]:.3f}, z={prediction[0, 2]:.3f}')
    
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
        
        return transformed_point
    
    def _create_pointcloud2_message(self, point):
        """Create a PointCloud2 message from a single 3D point."""
        # Create the point cloud message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Publish in map frame since point is already transformed

        # Use the already transformed point
        px = point[0]
        py = point[1]
        pz = point[2]
        
        # Define the point fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        # Set the point cloud properties
        msg.point_step = 12  # 3 floats * 4 bytes each
        msg.row_step = 12    # Only one point
        msg.height = 1
        msg.width = 1
        msg.is_dense = True
        
        # Pack the point data
        point_data = struct.pack('fff', px, py, pz)
        msg.data = point_data
        
        return msg

    def _create_obstacle_message(self, point):
        """Create an obstacle message from a single 3D point."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Always publish in 'map' frame

        # Use the already transformed point
        msg.pose.position.x = float(point[0])
        msg.pose.position.y = float(point[1])
        msg.pose.position.z = float(point[2])
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = EEPredictionModelNode()
    rclpy.spin(node)
    rclpy.shutdown()