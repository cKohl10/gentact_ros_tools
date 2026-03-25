#this script creates a pointcloud corresponding
#to each proximity sensor. the data outputted is 
#the distance from the proximity sensor / 4000 mm

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image 
from std_msgs.msg import Header, UInt16MultiArray, Bool
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2
import numpy as np 
import sys
import copy
import time
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from time import sleep


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.declare_parameter('num_sensors', 8)
        self.declare_parameter('publish_rate', 30.0)
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.pc_publisher = self.create_publisher(PointCloud2, 'sensor_pc_data', 10)
        self.pos_publisher = self.create_publisher(PointCloud2, 'sensor_pos', 10)
        self.status_pub = self.create_publisher(Bool, 'sensor_status_link5', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tof_subs = []
        self.pc_publishers = []
        self.closest_points = []
        
        for i in range(self.num_sensors):
            tof_topic = f'tof_msg_{i}'
           
            self.tof_subs.append(
                self.create_subscription(
                    UInt16MultiArray,
                    tof_topic,
                    self.make_tof_callback(i),
                    10
                )
            )
            self.pc_publishers.append(
                self.create_publisher(PointCloud2, f'sensor_pc_data_{i}', 10)
            )

        self.obstacle_pub = self.create_publisher(PoseStamped, '/obstacle_link5', 1)

        # Initialize closest points storage - one for each sensor
        self.closest_points = [None] * self.num_sensors
        
        obstacle_hz = self.publish_rate
        self.obstacle_timer = self.create_timer(1/obstacle_hz, self.obstacle_callback)
        self.status_pub.publish(Bool(data=True))
        self.heartbeat_timer = self.create_timer(3.0, self.heartbeat_publisher)
        self.status = True

    def heartbeat_publisher(self):
        self.status_pub.publish(Bool(data=self.status))
        self.status = False

    def obstacle_callback(self):
        # Find the closest point among all sensors
        valid_points = [point for point in self.closest_points if point is not None]
        
        if not valid_points:
            print("No valid points available yet")
            # sleep(3)
            return  # No valid points available yet

        # Convert to numpy array for easier processing
        points_array = np.array(valid_points)
        
        # Find the point with minimum distance (z-coordinate)
        closest_idx = np.argmin(points_array[:, 2])
        closest_point = points_array[closest_idx]

        try:
            map_transform = self.tf_buffer.lookup_transform(
                "map",
                f"link5_sensor_{closest_idx}",
                rclpy.time.Time()
            )

            transformed_point = self._transform_point(closest_point, map_transform)

            self.obstacle_msg = PoseStamped()
            self.obstacle_msg.header.stamp = self.get_clock().now().to_msg()
            self.obstacle_msg.header.frame_id = "map"
            self.obstacle_msg.pose.position.x = float(transformed_point[0])
            self.obstacle_msg.pose.position.y = float(transformed_point[1])
            self.obstacle_msg.pose.position.z = float(transformed_point[2])
            self.obstacle_pub.publish(self.obstacle_msg)
        except Exception as e:
            self.get_logger().error(f'Error in obstacle_callback: {e}')
            return

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

    def make_tof_callback(self, idx):
        def callback(msg):
            self.status = True
            
            try:
            
                # print(idx)
                # print(msg.data)
                data_grid_8x8 = np.array(msg.data, dtype=np.uint16).reshape(8, 8)
                #flips the array
                data_grid_8x8 = data_grid_8x8[::-1,:]

                #detection angle
                #view page 4 of https://www.st.com/resource/en/datasheet/vl53l5cx.pdf
                fov_angle = 45.0*(np.pi/180.0)
                mid_fov_angle = fov_angle/2.0
                angles_x = np.array([np.linspace(-mid_fov_angle, mid_fov_angle, 8)])
                angles_y = np.array([np.linspace(mid_fov_angle, -mid_fov_angle, 8)])
                angles_X, angles_Y = np.meshgrid(angles_x, angles_y)
                
                #normalize distance from sensor measurement by 4 m
                #b/c max distance sensor can read is 4m
                z_offset = (data_grid_8x8.astype(np.float64) / 1000) 
                z_offset[z_offset == 0.0] = 4.0
                x_offset, y_offset = self.calculate_grid_size(z_offset, angles_X, angles_Y)

                # FInd the closest point
                # closest_point = np.argmin(z_offset)
                closest_point = np.argmin(z_offset.flatten())

                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    #below is distance data (mm) from sensor
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
                ]

                #x_y_z_offset = np.dstack((x_offset, y_offset, z_offset, data_grid_8x8))
                x_y_z_offset = np.dstack((x_offset, y_offset, z_offset, z_offset))

                #8 row x 8 col = 64 resolution 
                sensor_pts = np.reshape(x_y_z_offset, (64, len(fields))).astype(np.float32)
                # Find the index of the closest point (smallest z value)
                closest_idx = np.argmin(sensor_pts[:, 2])
                closest_point = sensor_pts[closest_idx, :3]  # x, y, z
                self.closest_points[idx] = closest_point
                itemsize = sensor_pts.itemsize

                pc_msg = PointCloud2(
                    header=Header(frame_id=f'link5_sensor_{idx}'),
                    # header=Header(frame_id=f'link5_sensor_{idx}'),
                    height=1,
                    width=sensor_pts.shape[0],
                    is_dense=False,
                    is_bigendian=sys.byteorder != 'little',
                    fields=fields,
                    point_step=(itemsize * len(fields)),
                    #point_step=16, uncomment this if above doesn't work 
                    row_step = (itemsize * len(fields) * sensor_pts.shape[0]),
                    #row_step=16 * sensor_pts.shape[0], uncomment if above doesn't work 
                    data=sensor_pts.tobytes()
                )
                self.pc_publishers[idx].publish(pc_msg)
            except Exception as e:
                self.get_logger().error(f'Error in make_tof_callback: {e}')
                return

        return callback
    
    def calculate_grid_size(self, dist, angles_X, angles_Y):
        #dist = [8,8] dim list 

        #return x and y offsets of grid
        #this is going off of VL53L5CX FOV detection being 45
        x_pos = np.sin(angles_X)*dist    # angle is divided by 2 because solving sidelen of isoceles triangle 
        y_pos = np.sin(angles_Y)*dist

        

        return x_pos, y_pos

        
def main(args=None):

    
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()


    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()