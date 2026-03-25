import select
import socket
import struct
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float64MultiArray, Header, Int32MultiArray

IMAGE_WIDTH = 8
NUM_PIXELS = IMAGE_WIDTH * IMAGE_WIDTH


class UDP_PC_Publisher(Node):
    def _make_udp_socket(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
        sock.setblocking(False)
        sock.bind(("0.0.0.0", port))

        if self.multicast:
            mreq = struct.pack(
                "4sL", socket.inet_aton(self.multicast_group), socket.INADDR_ANY
            )
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            print(f"  [multicast] Joined group {self.multicast_group} on port {port}")
        return sock

    def __init__(self):
        super().__init__("udp_pointcloud_publisher")

        # Declare parameters
        self.declare_parameter("udp_port", 8888)
        self.declare_parameter("buffer_size", 4096)
        self.declare_parameter("timeout_seconds", 5.0)
        self.declare_parameter("link", "1")
        self.declare_parameter("num_sensors", 1)
        # self.declare_parameter('max_devices', 10)  # Maximum number of devices to track
        self.declare_parameter("unicast", False)
        self.declare_parameter("multicast", False)
        self.declare_parameter("multicast_group", None)

        # Get parameters
        self.udp_port = (
            self.get_parameter("udp_port").get_parameter_value().integer_value
        )
        self.buffer_size = (
            self.get_parameter("buffer_size").get_parameter_value().integer_value
        )
        self.timeout_seconds = (
            self.get_parameter("timeout_seconds").get_parameter_value().double_value
        )
        self.link = self.get_parameter("link").get_parameter_value().string_value
        self.link_num = "".join(filter(lambda x: x.isdigit(), self.link))
        self.num_sensors = (
            self.get_parameter("num_sensors").get_parameter_value().integer_value
        )
        # self.max_devices = self.get_parameter('max_devices').get_parameter_value().integer_value
        self.unicast = self.get_parameter("unicast").get_parameter_value().bool_value
        self.multicast = (
            self.get_parameter("multicast").get_parameter_value().bool_value
        )
        self.multicast_group = (
            self.get_parameter("multicast_group").get_parameter_value().string_value
        )

        # Device tracking
        self.device_publishers = {}  # device_id -> publisher
        self.device_last_seen = {}  # device_id -> last_receive_time
        self.device_receive_counts = {}  # device_id -> receive_count

        # UDP socket setup
        self.socket = self._make_udp_socket(self.udp_port)

        # Threading
        self.running = True
        self.receive_thread = threading.Thread(target=self._receive_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        # Create timer for status updates
        self.status_timer = self.create_timer(5.0, self._status_update)

        self.get_logger().info(f"UDP Sensor Publisher started on port {self.udp_port}")
        # self.get_logger().info(f"Supporting up to {self.max_devices} devices")

        # Precompute angle meshgrid (VL53L5CX 45° FoV, 8x8)
        fov_angle = 45.0 * (np.pi / 180.0)
        mid_fov_angle = fov_angle / 2.0
        angles_x = np.linspace(-mid_fov_angle, mid_fov_angle, 8)  # cols → sensor X
        angles_y = np.linspace(
            mid_fov_angle, -mid_fov_angle, 8
        )  # rows → sensor Y (row 0 = top)
        self.angles_X, self.angles_Y = np.meshgrid(angles_x, angles_y)

    def _get_or_create_publisher(self, sensor_id, device_id):
        """Get existing publisher or create new one for device"""
        if device_id not in self.device_publishers:
            # Create new publisher for this device
            topic_name = f"link{self.link}_sensor_{sensor_id}"
            # topic_name = f'link{self.link}_sensor_{sensor_id}'
            publisher = self.create_publisher(PointCloud2, topic_name, 1)
            device_id = f"link{self.link}_sensor_{sensor_id}"
            self.device_publishers[device_id] = publisher
            self.device_last_seen[device_id] = time.time()
            self.device_receive_counts[device_id] = 0

            self.get_logger().info(f"New device connected: {device_id} -> {topic_name}")

        return self.device_publishers[device_id]

    def _receive_loop(self):
        """Main receive loop running in separate thread"""
        while self.running and rclpy.ok():
            try:
                ready, _, _ = select.select([self.socket], [], [], 0.1)
                if not ready:
                    continue  # No data yet, loop back

                # Receive data
                data, addr = self.socket.recvfrom(self.buffer_size)
                if int(len(data)) < int(1 + NUM_PIXELS * 2):
                    print(f"length of data received is: {len(data)}")
                    continue

                # Parse sensor data
                sensor_data = self._parse_sensor_data(data)

                if sensor_data is not None:
                    device_id = sensor_data["device_id"]
                    sensor_id = sensor_data["sensor_id"]

                    # Get or create publisher for this device
                    publisher = self._get_or_create_publisher(sensor_id, device_id)

                    if publisher is not None:
                        # Publish to ROS2
                        self._publish_sensor_data(sensor_data, publisher, sensor_id)
                        print(f"published to {device_id}")

                        # Update status
                        self.device_last_seen[device_id] = time.time()
                        self.device_receive_counts[device_id] += 1

                        # Log received data
                        self.get_logger().debug(
                            f"Received from {addr}: device_id={device_id}, "
                            f"sensors={sensor_data['data']}, "
                            # f"raw_size={len(sensor_data['raw_data'])}"
                        )

            except socket.timeout:
                # Timeout is expected, continue
                continue
            except Exception as e:
                self.get_logger().error(f"Error in receive loop: {e}")
                continue

    def _parse_sensor_data(self, data):
        """
        Parse binary sensor data from ESP32

        Packet layout: [1 byte sensor ID] [64 uint16_t filtered] [64 uint16_t raw]
        """

        try:
            # Expected structure:
            # if len(data) < 4096 :  # Minimum size (4+4+4)
            #    self.get_logger().warn(f"Received data too small: {len(data)} bytes")
            #    return None

            # Unpack binary data
            sensor_id = data[0]
            mm = np.frombuffer(data[1 : 1 + NUM_PIXELS * 2], dtype=np.uint16) / 1000.0
            return {
                "sensor_id": sensor_id,
                "device_id": f"link{self.link}_sensor_{sensor_id}",
                "data": mm,
            }

        except struct.error as e:
            self.get_logger().error(f"Failed to parse sensor data: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Unexpected error parsing data: {e}")
            return None

    def calculate_grid_size(self, dist):

        # return x and y offsets of grid
        x_pos = np.sin(self.angles_X) * dist
        y_pos = np.sin(self.angles_Y) * dist
        return x_pos, y_pos

    def _publish_sensor_data(self, sensor_data, publisher, sensor_id):
        """Publish sensor data as ROS2 message"""
        try:
            data_grid_8x8 = sensor_data["data"].reshape(8, 8)
            print(data_grid_8x8)
            # flips the array -- dont need this anymore
            # data_grid_8x8 = data_grid_8x8[::-1, :]
            # flip horizontally
            data_grid_8x8 = data_grid_8x8[:, ::-1]

            # Detection angle
            # view page 4 of https://www.st.com/resource/en/datasheet/vl53l5cx.pdf

            # Convert to meters
            z_offset = data_grid_8x8.astype(np.float64)

            x_offset, y_offset = self.calculate_grid_size(z_offset)

            fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                # below is distance data (mm) from sensor
                PointField(
                    name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
                ),
            ]

            x_y_z_offset = np.dstack((x_offset, y_offset, z_offset, z_offset))

            # 8 row x 8 col = 64 resolution
            sensor_pts = np.reshape(x_y_z_offset, (64, len(fields))).astype(np.float32)
            itemsize = sensor_pts.itemsize

            # Create PointCloud2 msg
            pc_msg = PointCloud2(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=f"link{self.link}_sensor_{sensor_id}",
                ),
                height=1,
                width=sensor_pts.shape[0],
                is_dense=False,
                is_bigendian=sys.byteorder != "little",
                fields=fields,
                point_step=(itemsize * len(fields)),
                # point_step=16, uncomment this if above doesn't work
                row_step=(itemsize * len(fields) * sensor_pts.shape[0]),
                # row_step=16 * sensor_pts.shape[0], uncomment if above doesn't work
                data=sensor_pts.tobytes(),
            )
            publisher.publish(pc_msg)

            # Publish
            # publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing sensor data: {e}")

    def _status_update(self):
        """Periodic status update"""
        current_time = time.time()
        active_devices = 0
        total_packets = 0

        # Check each device
        for device_id in list(self.device_last_seen.keys()):
            time_since_last = current_time - self.device_last_seen[device_id]
            packet_count = self.device_receive_counts.get(device_id, 0)
            total_packets += packet_count

            if time_since_last > self.timeout_seconds:
                self.get_logger().warn(
                    f"Device {device_id}: No data for {time_since_last:.1f}s "
                    f"(total packets: {packet_count})"
                )
            else:
                active_devices += 1
                self.get_logger().info(
                    f"Device {device_id}: {packet_count} packets, "
                    f"last: {time_since_last:.1f}s ago"
                )

        if active_devices == 0:
            self.get_logger().warn(
                f"No active devices. Total packets received: {total_packets}"
            )
        else:
            self.get_logger().info(
                f"Active devices: {active_devices}/{len(self.device_publishers)}, "
                f"Total packets: {total_packets}"
            )

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.running = False
        if hasattr(self, "socket"):
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    publisher = UDP_PC_Publisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
