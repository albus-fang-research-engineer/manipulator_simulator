import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class URWaypointPublisher(Node):

    def __init__(self):
        super().__init__('ur_waypoint_publisher')

        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.timer = self.create_timer(2.0, self.publish_waypoint)

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

        point.positions = [0.0, -1.2, 1.3, -1.5, -1.57, 0.0]

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