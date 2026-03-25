# This is a machine learning model that predicts the end effector position of a robot arm based on the sensor data.
import rclpy
from rclpy.node import Node
import numpy as np
import os
import torch
import torch.nn as nn
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import PointCloud2, PointField
from ament_index_python.packages import get_package_share_directory
import struct
from sklearn.preprocessing import StandardScaler

class MLP(nn.Module):
    """Neural network model for end effector position prediction - matches training script."""
    
    def __init__(self, input_size=6, hidden_size=64, output_size=3):
        super(MLP, self).__init__()
        self.layer1 = nn.Linear(input_size, hidden_size)
        self.layer2 = nn.Linear(hidden_size, hidden_size)
        self.layer3 = nn.Linear(hidden_size, output_size)
        self.relu = nn.ReLU()
        
    def forward(self, x):
        x = self.relu(self.layer1(x))
        x = self.relu(self.layer2(x))
        x = self.layer3(x)
        return x

class EEPredictionModelNode(Node):
    def __init__(self):
        super().__init__('ee_prediction_model')
        
        # Load model from the trained .pth file
        # Use ROS2 parameter system with proper package path
        default_model_path = os.path.join(get_package_share_directory('gentact_ros_tools_hybrid'), 'config', 'mlp_model.pth')
        self.declare_parameter('model_path', default_model_path)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # Default configuration (will be overridden if model loads successfully)
        self.input_size = 6
        self.hidden_size = 64
        self.output_size = 3
        self.scaler_X = None
        self.scaler_y = None
        self.input_cols = None
        self.output_cols = None

        if os.path.exists(model_path):
            try:
                # Add safe globals for sklearn scalers
                torch.serialization.add_safe_globals([StandardScaler])
                
                checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)
                
                self.scaler_X = checkpoint.get('scaler_X')
                self.scaler_y = checkpoint.get('scaler_y')
                self.input_cols = checkpoint.get('input_cols', [f'sensor_{i}_cc' for i in range(6)])
                self.output_cols = checkpoint.get('output_cols', ['ee_x', 'ee_y', 'ee_z'])
                
                # Determine input size from scaler
                if self.scaler_X is not None:
                    self.input_size = self.scaler_X.n_features_in_
                
                # Initialize model with correct architecture
                self.model = MLP(
                    input_size=self.input_size,
                    hidden_size=self.hidden_size,
                    output_size=self.output_size
                )
                
                # Load trained weights
                self.model.load_state_dict(checkpoint['model_state_dict'])
                self.model.eval()
                
                self.get_logger().info(f'Model loaded successfully from {model_path}')
                self.get_logger().info(f'Input size: {self.input_size}, Output size: {self.output_size}')
                self.get_logger().info(f'Input columns: {self.input_cols}')
                self.get_logger().info(f'Output columns: {self.output_cols}')
                
            except Exception as e:
                self.get_logger().error(f'Error loading model: {e}')
                self._initialize_default_model()
        else:
            self.get_logger().warning(f'Model file {model_path} not found. Using untrained model.')
            self._initialize_default_model()

        self.sensor_sub = self.create_subscription(
            Int32MultiArray,
            '/sensor_raw',
            self.prediction_callback,
            1
        )
        self.prediction_pub = self.create_publisher(
            PointCloud2,
            '/ee_prediction',
            1
        )
        self.get_logger().info('EE Prediction Model Node Initialized')
    
    def _initialize_default_model(self):
        """Initialize a default untrained model."""
        self.model = MLP(
            input_size=self.input_size,
            hidden_size=self.hidden_size,
            output_size=self.output_size
        )
        self.model.eval()
        
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
        
        sensor_data = torch.tensor(sensor_data, dtype=torch.float32).unsqueeze(0)
        
        # Predict the end effector position
        with torch.no_grad():
            try:
                prediction = self.model(sensor_data)
                
                # Apply inverse scaling to get actual coordinates
                if self.scaler_y is not None:
                    prediction_np = prediction.numpy()
                    prediction_unscaled = self.scaler_y.inverse_transform(prediction_np)
                    prediction = torch.tensor(prediction_unscaled, dtype=torch.float32)
                
            except Exception as e:
                self.get_logger().error(f'Error making prediction: {e}')
                return
        
        # Convert the prediction to a PointCloud2 message
        point_cloud_msg = self._create_pointcloud2_message(prediction[0].numpy())
        
        self.prediction_pub.publish(point_cloud_msg)
        self.get_logger().debug(f'Published prediction: x={prediction[0, 0]:.3f}, y={prediction[0, 1]:.3f}, z={prediction[0, 2]:.3f}')
    
    def _create_pointcloud2_message(self, point):
        """Create a PointCloud2 message from a single 3D point."""
        # Create the point cloud message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "calibration_skin"  # Adjust frame_id as needed
        
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
        point_data = struct.pack('fff', point[0], point[1], point[2])
        msg.data = point_data
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = EEPredictionModelNode()
    rclpy.spin(node)
    rclpy.shutdown()