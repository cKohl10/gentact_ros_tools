import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gentact_ros_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'msg'), glob('gentact_ros_tools/msg/*.msg')),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='carson',
    maintainer_email='carson.kohlbrenner@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibration_direct_pub = gentact_ros_tools.calibration_direct_pub:main',
            'sensor_publisher = gentact_ros_tools.sensor_publisher:main',
            'sensor_subscriber = gentact_ros_tools.sensor_subscriber:main',
        ],
    },
)
