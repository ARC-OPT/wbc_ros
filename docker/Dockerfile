ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]
RUN sudo apt-get update

# Install WBC lib
RUN mkdir -p /arc-opt
WORKDIR /arc-opt
RUN sudo apt-get install wget && \
    wget https://raw.githubusercontent.com/ARC-OPT/wbc/master/scripts/install.sh && \
    sh install.sh

# Set up colcon workspace and install wbc_ros
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN vcs import src --input https://raw.githubusercontent.com/ARC-OPT/wbc_ros/${ROS_DISTRO}/repos.yaml && \
    rosdep install --from-paths src -i -r -y  && \
    source /opt/ros/${ROS_DISTRO}/setup.bash  && \
    colcon build --symlink-install  && \
    source install/setup.bash 
