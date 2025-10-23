import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gentact_ros_tools'

# Function to recursively get all files in a directory
def get_data_files_recursive(base_dir, target_dir):
    files = []
    for root, dirs, filenames in os.walk(base_dir):
        for filename in filenames:
            file_path = os.path.join(root, filename)
            # Calculate the relative path from base_dir
            rel_path = os.path.relpath(file_path, base_dir)
            # Create the target path
            target_path = os.path.join(target_dir, rel_path)
            files.append((os.path.dirname(target_path), [file_path]))
    return files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'msg'), glob('gentact_ros_tools/msg/*.msg')),
    ] + get_data_files_recursive('meshes', os.path.join('share', package_name, 'meshes')) + get_data_files_recursive('urdf', os.path.join('share', package_name, 'urdf')),
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'sensor_msgs', 'tf2_ros', 'geometry_msgs', 'tracikpy'],
    zip_safe=True,
    maintainer='carson',
    maintainer_email='carson.kohlbrenner@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'calibration_direct_pub = gentact_ros_tools.calibration_direct_pub:main',
            'sensor_publisher = gentact_ros_tools.sensor_publisher:main',
            'sensor_subscriber = gentact_ros_tools.sensor_subscriber:main',
            'capacitive_pcl = gentact_ros_tools.capacitive_pcl:main',
            'tuner = gentact_ros_tools.tuner:main',
            'sensor_tracking_pub = gentact_ros_tools.sensor_tracking_pub:main',
            'udp_sensor_publisher = gentact_ros_tools.udp_sensor_publisher:main',
            'processor = gentact_ros_tools.processor:main',
            'ik_solver = gentact_ros_tools.ik_solver:main',
            'training_data_processor = gentact_ros_tools.training_data_processor:main',
            'ee_prediction_model_mlp = gentact_ros_tools.ee_prediction_model_mlp:main',
            'ee_prediction_model_mamba = gentact_ros_tools.ee_prediction_model_mamba:main',
            'ee_prediction_verifier = gentact_ros_tools.ee_prediction_verifier:main',
            'panda2fr3 = gentact_ros_tools.panda2fr3:main',
            'fake_obj_pub = gentact_ros_tools.fake_obj_pub:main',
            'joint_state_test_pub = gentact_ros_tools.joint_state_test_pub:main',
            'franky_relay = gentact_ros_tools.franky_relay:main',
            'franky_xbox = gentact_ros_tools.franky_xbox:main',
            'closest_obstacle = gentact_ros_tools.closest_obstacle:main',
            'tof_pub_pc = gentact_ros_tools.tof_pub_pc:main',
            'franka_ros2_controller = gentact_ros_tools.franka_ros2_controller:main',
            'joint_states_monitor = gentact_ros_tools.joint_states_monitor:main',
            'test_talker = gentact_ros_tools.test_talker:main',
        ],
    },
)
