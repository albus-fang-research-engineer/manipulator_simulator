import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

import numpy as np
import pinocchio as pin


class TrajectoryVisualizer(Node):

    def __init__(self):

        super().__init__("trajectory_visualizer")

        self.get_logger().info("Starting trajectory visualizer (Pinocchio FK)")

        # ---------------------------------
        # Load URDF with Pinocchio
        # ---------------------------------

        urdf_path = "/manipulator_simulator/src/ur5e_rgbd/urdf/ur5e_rgbd.urdf"

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        for i, name in enumerate(self.model.names):
            print(i, name)
        self.nq = self.model.nq

        self.get_logger().info(f"Robot DOF: {self.nq}")

        # collect link frame names
        # self.link_names = []

        # for frame in self.model.frames:
        #     if frame.type == pin.FrameType.BODY:
        #         self.link_names.append(frame.name)
        self.link_names = [
            "base_link",
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "flange",
            "tool0",
        ]
        self.get_logger().info(f"Loaded {len(self.link_names)} link frames")

        # ---------------------------------
        # Subscriber
        # ---------------------------------

        self.sub = self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            self.traj_callback,
            10
        )

        # ---------------------------------
        # Publisher
        # ---------------------------------

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/robot_trajectory_markers",
            10
        )
        # ---------------------------------
        # assign fixed colors to links
        # ---------------------------------

        self.link_colors = {}

        rng = np.random.default_rng(42)   # fixed seed for reproducibility

        for link in self.link_names:
            color = rng.random(3)
            self.link_colors[link] = color
        # print("\nFrames in model:\n")

        # for i, frame in enumerate(self.model.frames):
        #     print(i, frame.name, frame.type)

    def traj_callback(self, msg):

        self.get_logger().info("Received trajectory")
        now = self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        markers = {}
        waypoint_marker = Marker()

        waypoint_marker.header.frame_id = "base_link"
        waypoint_marker.ns = "waypoints"
        waypoint_marker.id = 999

        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD

        waypoint_marker.scale.x = 0.03
        waypoint_marker.scale.y = 0.03
        waypoint_marker.scale.z = 0.03

        waypoint_marker.color.r = 1.0
        waypoint_marker.color.g = 0.2
        waypoint_marker.color.b = 0.2
        waypoint_marker.color.a = 1.0

        skeleton_marker = Marker()

        skeleton_marker.header.frame_id = "base_link"
        skeleton_marker.ns = "robot_skeleton"
        skeleton_marker.id = 2000

        skeleton_marker.type = Marker.LINE_LIST
        skeleton_marker.action = Marker.ADD

        skeleton_marker.scale.x = 0.01

        skeleton_marker.color.r = 0.9
        skeleton_marker.color.g = 0.9
        skeleton_marker.color.b = 0.9
        skeleton_marker.color.a = 0.5
        waypoint_marker.header.stamp = now
        skeleton_marker.header.stamp = now
        # ---------------------------------
        # initialize markers
        # ---------------------------------

        for i, link in enumerate(self.link_names):

            marker = Marker()

            marker.header.frame_id = "base_link"
            marker.ns = "robot_links"
            marker.id = i

            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.01

            color = self.link_colors[link]

            marker.color.r = float(color[0])
            marker.color.g = float(color[1])
            marker.color.b = float(color[2])
            marker.color.a = 1.0

            markers[link] = marker

        # ---------------------------------
        # iterate trajectory
        # ---------------------------------

        for point in msg.points:

            q = np.array(point.positions)

            # Pinocchio FK
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            # draw skeleton connections
            for i in range(len(self.link_names) - 1):

                frame1 = self.model.getFrameId(self.link_names[i])
                frame2 = self.model.getFrameId(self.link_names[i+1])

                pose1 = self.data.oMf[frame1]
                pose2 = self.data.oMf[frame2]

                p1 = Point()
                p1.x = float(pose1.translation[0])
                p1.y = float(pose1.translation[1])
                p1.z = float(pose1.translation[2])

                p2 = Point()
                p2.x = float(pose2.translation[0])
                p2.y = float(pose2.translation[1])
                p2.z = float(pose2.translation[2])

                skeleton_marker.points.append(p1)
                skeleton_marker.points.append(p2)
            for link in self.link_names:

                frame_id = self.model.getFrameId(link)
                pose = self.data.oMf[frame_id]

                p = Point()
                p.x = float(pose.translation[0])
                p.y = float(pose.translation[1])
                p.z = float(pose.translation[2])

                markers[link].points.append(p)
            frame_id = self.model.getFrameId("tool0")
            pose = self.data.oMf[frame_id]

            wp = Point()
            wp.x = float(pose.translation[0])
            wp.y = float(pose.translation[1])
            wp.z = float(pose.translation[2])

            waypoint_marker.points.append(wp)
        # ---------------------------------
        # publish markers
        # ---------------------------------

        for marker in markers.values():
            marker_array.markers.append(marker)
        marker_array.markers.append(waypoint_marker)
        marker_array.markers.append(skeleton_marker)
        self.marker_pub.publish(marker_array)
        
        self.get_logger().info("Published trajectory markers")


def main():

    rclpy.init()

    node = TrajectoryVisualizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()