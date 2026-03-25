import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tracikpy import TracIKSolver
import tf2_ros
import numpy as np
import os

def quaternion_to_rotation_matrix(q):
    """Convert quaternion [x, y, z, w] to 3x3 rotation matrix"""
    x, y, z, w = q
    
    # Normalize quaternion
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    if norm == 0:
        return np.eye(3)
    
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    
    # Convert to rotation matrix
    R = np.array([
        [1 - 2*y*y - 2*z*z,     2*x*y - 2*w*z,     2*x*z + 2*w*y],
        [    2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z,     2*y*z - 2*w*x],
        [    2*x*z - 2*w*y,     2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
    ])
    
    return R

def quaternion_to_matrix(q):
    """Convert quaternion [x, y, z, w] to 4x4 homogeneous transformation matrix"""
    R = quaternion_to_rotation_matrix(q)
    mat = np.eye(4)
    mat[:3, :3] = R
    return mat

class FrankaIKSolver(Node):
    def __init__(self):
        super().__init__('franka_ik_solver')

        self.declare_parameter('base_link', 'fr3_link0')
        self.declare_parameter('tip_link', 'end_effector_tip')
        self.declare_parameter('urdf_path', '')

        base_link = self.get_parameter('base_link').get_parameter_value().string_value
        tip_link = self.get_parameter('tip_link').get_parameter_value().string_value
        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value

        if not urdf_path or not os.path.exists(urdf_path):
            self.get_logger().fatal(f'URDF path "{urdf_path}" not set or file does not exist.')
            self.destroy_node()
            return

        try:
            self.ik_solver = TracIKSolver(urdf_path, base_link, tip_link)
        except Exception as e:
            self.get_logger().fatal(f"Failed to create TracIKSolver: {e}")
            self.destroy_node()
            return

        # Create publishers and subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_sub = self.create_subscription(
            TransformStamped,
            '/target_pose',
            self.pose_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def pose_callback(self, msg: TransformStamped):
        try:
            # Lookup transform from map to the target pose frame
            transform_to_base = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time()
            )

            pose_mat = self._transform_to_matrix(msg.transform)
            base_mat = self._transform_to_matrix(transform_to_base.transform)
            full_mat = np.matmul(base_mat, pose_mat)

            # Solve IK
            seed = np.zeros(self.ik_solver.number_of_joints)
            ik_solution = self.ik_solver.ik(full_mat, qinit=seed)

            if ik_solution is not None:
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name = [f'fr3_joint{i+1}' for i in range(self.ik_solver.number_of_joints)]
                js.position = list(ik_solution)
                js.velocity = [0.0] * self.ik_solver.number_of_joints
                js.effort = [0.0] * self.ik_solver.number_of_joints
                
                self.joint_pub.publish(js)

        except Exception as e:
            self.get_logger().error(f'Error in pose_callback: {e}')

    def _transform_to_matrix(self, t: TransformStamped.transform.__class__):
        q = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        mat = quaternion_to_matrix(q)
        mat[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return mat

def main(args=None):
    rclpy.init(args=args)
    node = FrankaIKSolver()
    rclpy.spin(node)
    rclpy.shutdown()