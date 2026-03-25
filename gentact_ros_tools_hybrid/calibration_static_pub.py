#!/usr/bin/env python3

import sys
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf_transformations

def main(args=None):
    if len(sys.argv) != 2:
        print("Usage: python3 calibration_static_pub.py <json_file>")
        return

    rclpy.init(args=args)
    node = Node('calibration_static_pub')
    
    with open(sys.argv[1], 'r') as f:
        data = json.load(f)
    
    pose = data['pose']
    position = pose['position']
    orientation = pose['orientation']
    
    quaternion = tf_transformations.quaternion_from_euler(
        orientation[0], orientation[1], orientation[2]
    )
    
    broadcaster = StaticTransformBroadcaster(node)
    
    transform = TransformStamped()
    transform.header.stamp = node.get_clock().now().to_msg()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'calibration_frame'
    transform.transform.translation.x = position[0]
    transform.transform.translation.y = position[1]
    transform.transform.translation.z = position[2]
    transform.transform.rotation.x = quaternion[0]
    transform.transform.rotation.y = quaternion[1]
    transform.transform.rotation.z = quaternion[2]
    transform.transform.rotation.w = quaternion[3]
    
    broadcaster.sendTransform(transform)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
