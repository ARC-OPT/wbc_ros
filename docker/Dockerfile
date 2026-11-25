FROM ros:jazzy AS base

RUN sudo apt update && sudo apt install wget

WORKDIR /root/arc-opt
RUN wget https://raw.githubusercontent.com/ARC-OPT/wbc/master/scripts/full_install.sh
RUN sh full_install.sh

WORKDIR /root/ros2_ws
RUN mkdir src
RUN vcs import src --input https://raw.githubusercontent.com/ARC-OPT/wbc_ros/main/repos.yaml
# Install the dependencies of all packages (including wbc_ros), then drop wbc_ros:
# the devcontainer bind-mounts the local wbc_ros checkout to src/wbc_ros.
RUN rosdep install --from-paths src -i -r -y && rm -rf src/wbc_ros
RUN . /opt/ros/jazzy/setup.sh && colcon build --symlink-install

# Source ROS and the workspace in every interactive shell
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash' >> /root/.bashrc
