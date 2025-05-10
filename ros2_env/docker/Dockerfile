FROM osrf/ros:humble-desktop

# Set the environment variable to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && \
    apt-get install -y build-essential nano \
    ros-humble-desktop-full net-tools \
    tmux \
    iputils-ping && \
    rm -rf /var/lib/apt/lists/*
    
# Install additional ROS Humble packages
RUN apt-get update && \
    apt-get install -y net-tools iproute2 ros-humble-plotjuggler* ros-humble-foxglove-bridge ros-humble-turtlesim* gedit && \
    rm -rf /var/lib/apt/lists/*

# Create a non-root user named 'ros' with home directory
RUN useradd -m -s /bin/bash ros && \
    mkdir -p /home/ros/data && \
    mkdir -p /home/ros/ros2_ws/src && \
    chown -R ros:ros /home/ros

# Switch to the ros user
USER ros
WORKDIR /home/ros

# Source ROS environment for the user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && echo "source /home/ros/ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]