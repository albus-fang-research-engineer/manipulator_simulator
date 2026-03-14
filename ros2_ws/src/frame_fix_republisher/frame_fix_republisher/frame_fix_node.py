import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from rclpy.qos import qos_profile_sensor_data


class FrameFixRepublisher(Node):

    def __init__(self):
        super().__init__('frame_fix_republisher')

        self.target_frame = "camera_link"

        # INPUT TOPICS (Gazebo)
        points_in = "/world/camera_world/model/ur/link/wrist_3_link/sensor/rgbd_camera/points"
        depth_in = "/world/camera_world/model/ur/link/wrist_3_link/sensor/rgbd_camera/depth_image"
        image_in = "/world/camera_world/model/ur/link/wrist_3_link/sensor/rgbd_camera/image"
        info_in = "/world/camera_world/model/ur/link/wrist_3_link/sensor/rgbd_camera/camera_info"

        # OUTPUT TOPICS (clean)
        points_out = "/camera/points"
        depth_out = "/camera/depth_image"
        image_out = "/camera/image"
        info_out = "/camera/camera_info"

        # Subscribers
        self.create_subscription(PointCloud2, points_in, self.points_cb, qos_profile_sensor_data)
        self.create_subscription(Image, depth_in, self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(Image, image_in, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_in, self.info_cb, qos_profile_sensor_data)

        # Publishers
        self.points_pub = self.create_publisher(PointCloud2, points_out, qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, depth_out, qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, image_out, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, info_out, qos_profile_sensor_data)

        self.get_logger().info("Frame fix republisher started")

    def fix_frame(self, msg):
        msg.header.frame_id = self.target_frame
        return msg

    def points_cb(self, msg):
        self.points_pub.publish(self.fix_frame(msg))

    def depth_cb(self, msg):
        self.depth_pub.publish(self.fix_frame(msg))

    def image_cb(self, msg):
        self.image_pub.publish(self.fix_frame(msg))

    def info_cb(self, msg):
        self.info_pub.publish(self.fix_frame(msg))


def main():
    rclpy.init()
    node = FrameFixRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()