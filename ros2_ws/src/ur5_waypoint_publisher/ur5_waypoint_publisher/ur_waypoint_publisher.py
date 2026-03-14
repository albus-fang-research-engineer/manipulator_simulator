import rclpy
from rclpy.node import Node
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class URWaypointPublisher(Node):

    def __init__(self):
        super().__init__('ur_waypoint_publisher')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # self.timer = self.create_timer(2.0, self.publish_waypoint)
        # self.timer = self.create_timer(5.0, self.publish_trajectory)
        self.timer = self.create_timer(5.0, self.publish_small_step_trajectory)
    def publish_small_step_trajectory(self):

        msg = JointTrajectory()

        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # upright starting pose
        q = np.array([
            0.0,
            -1.57,
            0.0,
            -1.57,
            0.0,
            0.0
        ])

        num_points = 20
        step = np.array([
            0.03,  # shoulder_pan
            -0.03,  # shoulder_lift
            -0.03,  # elbow
            -0.03,  # wrist_1
            0.03,  # wrist_2
            -0.03   # wrist_3
        ])
        dt = 0.1

        for i in range(num_points):

            p = JointTrajectoryPoint()

            # subtract small amount each step
            q = q + step

            p.positions = q.tolist()
            p.velocities = [0.0] * 6

            t = i * dt
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t % 1) * 1e9)

            msg.points.append(p)

        self.publisher.publish(msg)

        self.get_logger().info("Published 100-point smooth trajectory")

        # run only once
        self.timer.cancel()
    def publish_trajectory(self):

        msg = JointTrajectory()

        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # ---- trajectory points ----

        p1 = JointTrajectoryPoint()
        p1.positions = [0.0, 0.16, -0.2, -1.2, -1.0, -1.0]
        p1.time_from_start.sec = 2

        p2 = JointTrajectoryPoint()
        p2.positions = [0.3, 0.3, -0.5, -1.0, -1.2, -0.8]
        p2.time_from_start.sec = 4

        p3 = JointTrajectoryPoint()
        p3.positions = [-0.2, 0.5, -0.4, -1.5, -1.0, -0.5]
        p3.time_from_start.sec = 6

        msg.points.append(p1)
        msg.points.append(p2)
        msg.points.append(p3)

        self.publisher.publish(msg)

        self.get_logger().info("Trajectory sent!")

    def publish_waypoint(self):

        msg = JointTrajectory()

        msg.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()

        point.positions = [0.0, 0.16, -0.2, -1.2, -1.0, -1.0]

        point.time_from_start.sec = 3

        msg.points.append(point)

        self.publisher.publish(msg)

        self.get_logger().info("Waypoint sent!")


def main(args=None):
    rclpy.init(args=args)

    node = URWaypointPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()