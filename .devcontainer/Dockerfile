FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# Base tools
# -----------------------------
RUN apt-get update && apt-get install -y \
    git \
    nano \
    wget \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Gazebo + ROS integration
# -----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# MoveIt (for manipulators)
# -----------------------------
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-servo \
    && rm -rf /var/lib/apt/lists/*
RUN apt update && apt install -y \
    ros-humble-ur \
    ros-humble-ur-description \
    ros-humble-ur-robot-driver \
    ros-humble-ur-moveit-config \
    ros-humble-ur-simulation-gz

RUN apt update && apt install ros-humble-realsense2-description -y
# -----------------------------
# Initialize rosdep
# -----------------------------
RUN rosdep init || true
RUN rosdep update



# -----------------------------
# Setup bash
# -----------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]