import argparse
import copy
import os

import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
import tf2_sensor_msgs
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header


def load_config(config_file_name):
    package_share = get_package_share_directory("gentact_ros_tools_hybrid")
    config_file = os.path.join(package_share, "config", config_file_name)
    with open(config_file, "r") as file:
        return yaml.safe_load(file)


def build_topics(config):
    topics = []
    try:
        for link_key, sensor_config in config["sensors"].items():
            if isinstance(sensor_config, dict) and sensor_config.get("active", False):
                if sensor_config.get("type", "") == "SPAD":
                    for sensor_key in range(sensor_config.get("num_sensors", 0)):
                        link_num = [int(c) for c in link_key if c.isdigit()][0]
                        if link_num == 5:
                            if "back" in link_key:
                                topics.append(
                                    f"/link{link_num}_back_sensor_{sensor_key}"
                                )
                            else:
                                topics.append(
                                    f"/link{link_num}_front_sensor_{sensor_key}"
                                )
                        else:
                            topics.append(f"/link{link_num}_sensor_{sensor_key}")
                        print(f"Adding topic: {topics[-1]}")
    except KeyError:
        raise KeyError("No sensors in config file")
    return topics


class PointCloudMerger(Node):
    def __init__(self):
        super().__init__("pointcloud_merger")

        parser = argparse.ArgumentParser()
        parser.add_argument("--config", type=str, default="hybrid.yaml")
        args, _ = parser.parse_known_args()
        self.config = load_config(args.config)

        self.target_frame = "base"
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=10.0)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(PointCloud2, "/cloud_merged", 10)

        self.clouds = {}
        topics = build_topics(self.config)
        self.expected_count = len(topics)
        self.expected_count = 27

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        for t in topics:
            self.create_subscription(
                PointCloud2,
                t,
                lambda msg, topic=t: self.cloud_cb(msg, topic),
                qos,
            )

        # Merge at 10 Hz regardless of individual topic rates
        self.create_timer(0.1, self.merge_and_publish)

    def cloud_cb(self, msg, topic):
        self.clouds[topic] = msg

    def merge_and_publish(self):
        if len(self.clouds) < self.expected_count:
            self.get_logger().info(
                f"Waiting for clouds: {len(self.clouds)}/{self.expected_count}",
                throttle_duration_sec=2.0,
            )
            return

        use_sim_time = (
            self.get_parameter("use_sim_time").get_parameter_value().bool_value
        )
        all_points = []

        for topic, cloud_msg in self.clouds.items():
            try:
                if use_sim_time:
                    cloud_copy = copy.deepcopy(cloud_msg)
                    cloud_copy.header.stamp = Time().to_msg()
                    transformed = self.tf_buffer.transform(
                        cloud_copy,
                        self.target_frame,
                        timeout=rclpy.duration.Duration(seconds=0.1),
                    )
                else:
                    transformed = self.tf_buffer.transform(
                        cloud_msg,
                        self.target_frame,
                        timeout=rclpy.duration.Duration(seconds=0.1),
                    )
            except Exception as e:
                self.get_logger().warn(
                    f"TF transform failed for {topic}: {e}",
                    throttle_duration_sec=1.0,
                )
                continue

            pts = list(
                pc2.read_points(
                    transformed, skip_nans=True, field_names=("x", "y", "z")
                )
            )
            all_points.extend(pts)

        if not all_points:
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        merged = pc2.create_cloud_xyz32(header, all_points)
        self.publisher.publish(merged)


def main():
    rclpy.init()
    rclpy.spin(PointCloudMerger())


if __name__ == "__main__":
    main()
