#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
import torch

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# import your libraries
from chance_constrained_planning_6d import solve_step_ur5e
from chance_constrained_planning_6d import rollout_optimized

# your FK + Jacobian
from ur5e_kinematics import fk_ur5e, jacobian_ur5e

# your neural model loader
from load_njsdf.inference import load_model


class ChanceConstrainedOptimizer(Node):

    def __init__(self):
        super().__init__("chance_constrained_optimizer")

        # -------------------------
        # Parameters
        # -------------------------

        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("output_folder", "/tmp/chance_debug")

        model_path = self.get_parameter("model_path").value
        device_name = self.get_parameter("device").value
        self.folder = self.get_parameter("output_folder").value

        self.device = torch.device(device_name)

        # -------------------------
        # Load NN distance model
        # -------------------------

        self.model = load_model(model_path, self.device)

        # -------------------------
        # Obstacles (placeholder)
        # -------------------------

        self.obstacle_points = np.zeros((1, 3))  # replace with real cloud

        # -------------------------
        # ROS2 Interfaces
        # -------------------------

        self.sub = self.create_subscription(
            JointTrajectory,
            "/nominal_joint_trajectory",
            self.nominal_callback,
            10,
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            "/optimized_joint_trajectory",
            10,
        )

        self.get_logger().info("Chance Constrained Optimizer Node Started")

    # -------------------------------------------------------
    # Solver wrapper
    # -------------------------------------------------------

    def solver(self, q0, q_goal, obstacle_points, model, device, epoch, folder):

        q_next, debug = solve_step_ur5e(
            q0=q0,
            x_goal=fk_ur5e(q_goal),
            obstacle_points=obstacle_points,
            model=model,
            device=device,
            fk_fn=fk_ur5e,
            jacobian_fn=jacobian_ur5e,
            epoch=epoch,
            folder=folder,
        )

        mu = debug["mu"]
        sigma = debug["sigma"]

        return q_next, mu, sigma

    # -------------------------------------------------------
    # Callback
    # -------------------------------------------------------

    def nominal_callback(self, msg):

        self.get_logger().info("Received nominal trajectory")

        # convert trajectory to numpy list
        path = []

        for pt in msg.points:
            q = np.array(pt.positions)
            path.append(q)

        start = path[0]

        # -------------------------
        # Run optimizer
        # -------------------------

        traj = rollout_optimized(
            start=start,
            path=path[1:],
            obstacle_points=self.obstacle_points,
            solver=self.solver,
            model=self.model,
            device=self.device,
            epoch=0,
            folder=self.folder,
        )

        # -------------------------
        # Publish result
        # -------------------------

        out_msg = JointTrajectory()

        out_msg.joint_names = msg.joint_names

        for q in traj:

            pt = JointTrajectoryPoint()
            pt.positions = q.tolist()
            pt.time_from_start = rclpy.duration.Duration(seconds=0.1).to_msg()

            out_msg.points.append(pt)

        self.pub.publish(out_msg)

        self.get_logger().info("Published optimized trajectory")


def main(args=None):

    rclpy.init(args=args)

    node = ChanceConstrainedOptimizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()