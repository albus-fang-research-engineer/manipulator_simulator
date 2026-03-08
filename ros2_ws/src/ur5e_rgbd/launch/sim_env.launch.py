from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():

    # -------------------------------------------------
    # Paths
    # -------------------------------------------------

    ur_sim_pkg = FindPackageShare("ur_simulation_gz")

    ur_sim_launch = PathJoinSubstitution([
        ur_sim_pkg,
        "launch",
        "ur_sim_control.launch.py"
    ])

    description_file = "/manipulator_simulator/src/ur5e_rgbd/urdf/ur5e_rgbd.xacro"
    world_file = "/manipulator_simulator/src/ur5e_rgbd/sim_env/camera_world.sdf"

    # -------------------------------------------------
    # Launch UR + Gazebo simulation
    # -------------------------------------------------

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_sim_launch),
        launch_arguments={
            "ur_type": "ur5e",
            "description_file": description_file,
            "world_file": world_file,
        }.items(),
    )

    # -------------------------------------------------
    # Gazebo → ROS2 bridge for camera
    # -------------------------------------------------

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/camera_world/model/ur/link/wrist_3_link/sensor/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/world/camera_world/model/ur/link/wrist_3_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/world/camera_world/model/ur/link/wrist_3_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/world/camera_world/model/ur/link/wrist_3_link/sensor/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    # -------------------------------------------------

    return LaunchDescription([
        ur_sim,
        camera_bridge
    ])