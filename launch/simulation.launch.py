#  Copyright (c) 2025 Franka Robotics GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

############################################################################
# Parameters:
# arm_id: ID of the type of arm used (default: '')
# arm_prefix: Prefix for arm topics (default: '')
# namespace: Namespace for the robot (default: '')
# urdf_file: URDF file path relative to franka_description/robots (default: 'fr3/fr3.urdf.xacro')
# robot_ip: Hostname or IP address of the robot (default: '172.16.0.3')
# load_gripper: Use Franka Gripper as an end-effector (default: 'false')
# use_fake_hardware: Use fake hardware (default: 'false')
# fake_sensor_commands: Fake sensor commands (default: 'false')
# joint_state_rate: Rate for joint state publishing in Hz (default: '30')
#
# The franka.launch.py launch file provides a robust and flexible interface
# for launching core Franka Robotics components, including robot_state_publisher,
# ros2_control_node, joint_state_publisher, joint_state_broadcaster,
# franka_robot_state_broadcaster, and optionally franka_gripper, with support
# for both namespaced and non-namespaced environments.
# Example:
# ros2 launch franka_bringup franka.launch.py arm_id:=fr3 namespace:=NS1 robot_ip:=172.16.0.3

# This is an error prone commandline, you may prefer to write the parameters into a YAML file like:
#   franka_bringup/config/franka.config.yaml
# That is especially useful if you want to use multiple namespaces.
# In that case, it's not possible to specify the parameters on the command line,
# since each parameter would have to be somehow isolated or prefixed by the namespace.
# then later parsed by the launch file.
# See: example.launch.py for more details.
#
# You may wish to experiment with the namespace parameter to see how it affects topic names
# and service names. The default namespace is empty, which means that the
# topics and services are not namespaced. If you set the namespace to 'franka1',
# the topics and services will be namespaced with 'franka1'. For example, the
# joint_state_publisher will publish to '/franka1/joint_states' instead of '/joint_states'.
# and the controller_manager will look for the controllers in the 'franka1' namespace.
# To see the difference you can run the following command:
#   ros2 topic list | grep joint_states
#   ros2 service list | grep controller_manager
# This becomes usefull for example when you require multiple Franka robots doing
# possibly different but related tasks. So, you might have the "PICK" and "PLACE" robots
# in the same workspace, but they are not supposed to interfere with each other.
#
# This script generates a URDF file using the specified xacro file to configure the robot
# description and integrates with controllers.yaml for controller management.
# It is designed to be called (included) by higher-level launch files, such as example.launch.py,
# which will, by default, rely upon franka.config.yaml for robot-specific parameters.
# RViz is not launched by this script but can be included by higher-level launch files
# if use_rviz is enabled. Ensure urdf_file parameter (a xacro file) exists in
# franka_description/robots to avoid runtime errors.
#
# This approach improves upon earlier launch scripts, which often lacked namespace
# support and were less modular, offering a more consistent and maintainable solution.
# While some may prefer the older scripts for their simplicity in specific scenarios,
# franka.launch.py enhances flexibility and scalability for diverse Franka Robotics
# applications.
############################################################################
import os

from ament_index_python.packages import get_package_share_directory


import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Generates the "default" nodes (controller_manager, robot_state_publisher, etc.)
# for the Franka robot. This function is called by the main launch file.
# It uses the xacro library to process the URDF file and generate the robot description.


def generate_robot_nodes(context):
    arm_id_str = LaunchConfiguration('arm_id').perform(context)
    robot_urdf_file = LaunchConfiguration('urdf_file').perform(context)
    ee_id_str = LaunchConfiguration('ee_id').perform(context)
    
    franka_xacro_file = os.path.join(
        get_package_share_directory('gentact_descriptions'),
        'robots',
        robot_urdf_file,
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': LaunchConfiguration('load_gripper').perform(context),
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': ee_id_str,
            'link1_skin': LaunchConfiguration('link1_skin').perform(context),
            'link2_skin': LaunchConfiguration('link2_skin').perform(context),
            'link3_skin': LaunchConfiguration('link3_skin').perform(context),
            'link4_skin': LaunchConfiguration('link4_skin').perform(context),
            'link5_skin': LaunchConfiguration('link5_skin').perform(context),
            'link6_skin': LaunchConfiguration('link6_skin').perform(context),
        }
    ).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_config},
            {'use_sim_time': True},  # Important for Gazebo simulation
        ]
    )

    return [robot_state_publisher]

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('arm_id',
                              default_value='fr3',
                              description='ID of the type of arm used'),
        DeclareLaunchArgument('arm_prefix',
                              default_value='',
                              description='Prefix for arm topics'),
        DeclareLaunchArgument('ee_id',
                              default_value='franka_hand',
                              description='ID of the end-effector'),
        DeclareLaunchArgument('namespace',
                              default_value='',
                              description='Namespace for the robot'),
        DeclareLaunchArgument('urdf_file',
                              default_value='fr3/fr3_full_skin.urdf.xacro',
                              description='Path to URDF file'),
        DeclareLaunchArgument('load_gripper',
                              default_value='true',
                              description='Use Franka Gripper as an end-effector'),
        DeclareLaunchArgument('joint_state_rate',
                              default_value='30',
                              description='Rate for joint state publishing in Hz'),
        DeclareLaunchArgument('link1_skin',
                              default_value='',
                              description='Link 1 skin'),
        DeclareLaunchArgument('link2_skin',
                              default_value='',
                              description='Link 2 skin'),
        DeclareLaunchArgument('link3_skin',
                              default_value='',
                              description='Link 3 skin'),
        DeclareLaunchArgument('link4_skin',
                              default_value='',
                              description='Link 4 skin'),
        DeclareLaunchArgument('link5_skin',
                              default_value='',
                              description='Link 5 skin'),
        DeclareLaunchArgument('link6_skin',
                              default_value='',
                              description='Link 6 skin'),
    ]

    namespace = LaunchConfiguration('namespace')

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('gentact_descriptions'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Use spawner so we can pass a param file to the controller
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('hiro_collision_avoidance_ros2'),
        'config',
        'controllers.yaml',
    ])

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    hiro_joint_velocity_example_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hiro_joint_velocity_example_controller',
            '--controller-manager-timeout', '30',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    joint_state_publisher_sources = ['franka/joint_states', 'franka_gripper/joint_states']
    joint_state_rate = LaunchConfiguration('joint_state_rate')
    
    return LaunchDescription(launch_args + [
        OpaqueFunction(function=generate_robot_nodes),
        gazebo_empty_world,
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[hiro_joint_velocity_example_controller],
            )
        ),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     namespace=namespace,
        #     parameters=[{
        #         'source_list': joint_state_publisher_sources,
        #         'rate': joint_state_rate,
        #         'use_robot_description': False,
        #     }],
        #     output='screen',
        # ),
    ])