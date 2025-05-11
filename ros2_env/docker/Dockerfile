FROM osrf/ros:humble-desktop

# Set the environment variable to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Switch to root user to install packages
USER root

# Install necessary packages in a single RUN to reduce layers
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    nano \
    ros-humble-desktop-full \ 
    net-tools \
    tmux \
    iputils-ping \
    iproute2 \
    ros-humble-plotjuggler* \
    ros-humble-foxglove-bridge \
    ros-humble-turtlesim* \
    gedit \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-gazebo-ros-pkgs \ 
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user named 'ros' with home directory
RUN id -u ros || useradd -m -s /bin/bash ros && \
    mkdir -p /home/ros/data && \
    mkdir -p /home/ros/ros2_ws/src && \
    chown -R ros:ros /home/ros

# Switch to the ros user
USER ros
WORKDIR /home/ros

# Source ROS environment for the user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo 'alias ls="ls --color=auto"' >> ~/.bashrc && \
    echo 'alias ll="ls -la"' >> ~/.bashrc

# Default command
CMD ["bash"]