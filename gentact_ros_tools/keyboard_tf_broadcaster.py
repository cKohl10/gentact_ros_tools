import sys
import select
import termios
import tty
import yaml
import math
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

HELP_TEXT = (
    "Controls (translation | rotation):\n"
    "  w/s: +x/-x          |  i/k: +pitch/-pitch\n"
    "  a/d: +y/-y          |  j/l: +yaw/-yaw\n"
    "  r/f: +z/-z          |  u/o: +roll/-roll\n"
    "Steps:\n"
    "  t/g: inc/dec translation step\n"
    "  y/h: inc/dec rotation step (deg)\n"
    "Misc:\n"
    "  space: reset pose\n"
    "  ?: show help\n"
    "  q: quit\n"
)


class KeyboardTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__('keyboard_tf_broadcaster')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'calibration_base')
        self.declare_parameter('translation_step', 0.01)
        self.declare_parameter('rotation_step_deg', 5.0)
        self.declare_parameter('config', '')

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.translation_step = self.get_parameter('translation_step').get_parameter_value().double_value
        self.rotation_step_deg = self.get_parameter('rotation_step_deg').get_parameter_value().double_value
        self.config = self.get_parameter('config').get_parameter_value().string_value
        self.tf_broadcaster = TransformBroadcaster(self)

        self.start_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        print(f"Config parameter received: '{self.config}'")
        print(f"Config parameter length: {len(self.config)}")
        print(f"Config parameter repr: {repr(self.config)}")
        
        package_share_directory = get_package_share_directory('gentact_ros_tools')
        config_path = os.path.join(package_share_directory, 'config', self.config)
        print(f"Loading config from {config_path}")

        if self.config and self.config.strip():
            try:
                with open(config_path, 'r') as f:
                    config_data = yaml.safe_load(f)
                if 'calibration' in config_data and 'transform' in config_data['calibration']:
                    transform = config_data['calibration']['transform']
                    self.start_pose = [float(transform[i]) for i in range(6)]
                    if len(transform) > 6:
                        self.parent_frame = str(transform[6])
                    if len(transform) > 7:
                        self.child_frame = str(transform[7])
            except Exception as e:
                self.get_logger().warn(f"Failed to load config file '{self.config}': {e}")

        self.x = self.start_pose[0]
        self.y = self.start_pose[1]
        self.z = self.start_pose[2]
        self.yaw = self.start_pose[3]
        self.pitch = self.start_pose[4]
        self.roll = self.start_pose[5]

        self.print_status(show_help=True)

    def print_status(self, show_help: bool = False) -> None:
        if show_help:
            print(HELP_TEXT)
        print(
            f"parent='{self.parent_frame}', child='{self.child_frame}', "
            f"dxyz={self.translation_step:.4f} m, dRPY={self.rotation_step_deg:.2f} deg"
        )
        print(
            f"Pose: x={self.x:.4f} y={self.y:.4f} z={self.z:.4f} "
            f"roll={self.roll:.3f} pitch={self.pitch:.3f} yaw={self.yaw:.3f} (rad)"
        )
        print(f"YPR: [{self.x}, {self.y}, {self.z}, {self.yaw}, {self.pitch}, {self.roll}]")
        print(f"static_transform_publisher: {self.x} {self.y} {self.z} {self.yaw} {self.pitch} {self.roll} {self.parent_frame} {self.child_frame}")

    def send_tf(self) -> None:
        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = float(self.z)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        self.tf_broadcaster.sendTransform(t)

    def handle_key(self, key: str) -> bool:
        if key == 'q':
            return False
        if key == ' ':
            self.x = self.y = self.z = 0.0
            self.roll = self.pitch = self.yaw = 0.0
            self.print_status()
            self.send_tf()
            return True
        if key == '?':
            self.print_status(show_help=True)
            return True

        moved = False
        s = self.translation_step
        r = self.rotation_step_deg * 3.141592653589793 / 180.0

        if key == 'w':
            self.x += s; moved = True
        elif key == 's':
            self.x -= s; moved = True
        elif key == 'a':
            self.y += s; moved = True
        elif key == 'd':
            self.y -= s; moved = True
        elif key == 'r':
            self.z += s; moved = True
        elif key == 'f':
            self.z -= s; moved = True
        elif key == 'j':
            self.yaw += r; moved = True
        elif key == 'l':
            self.yaw -= r; moved = True
        elif key == 'i':
            self.pitch += r; moved = True
        elif key == 'k':
            self.pitch -= r; moved = True
        elif key == 'u':
            self.roll += r; moved = True
        elif key == 'o':
            self.roll -= r; moved = True
        elif key == 't':
            self.translation_step *= 1.5; self.print_status(); return True
        elif key == 'g':
            self.translation_step = max(self.translation_step / 1.5, 1e-6); self.print_status(); return True
        elif key == 'y':
            self.rotation_step_deg *= 1.5; self.print_status(); return True
        elif key == 'h':
            self.rotation_step_deg = max(self.rotation_step_deg / 1.5, 1e-4); self.print_status(); return True

        if moved:
            self.print_status()
            self.send_tf()
        return True


def get_key(timeout_sec: float = 0.1) -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if rlist:
            ch = sys.stdin.read(1)
        else:
            ch = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardTfBroadcaster()
    try:
        running = True
        node.send_tf()
        while rclpy.ok() and running:
            key = get_key(0.1)
            if key:
                running = node.handle_key(key)
        print('Exiting.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


