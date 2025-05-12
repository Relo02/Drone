FROM ros:humble-perception-jammy

# Set the environment variable to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary packages
RUN apt-get update && \
    apt-get install -y build-essential nano \
    ros-humble-desktop-full net-tools \
    tmux \
    iputils-ping && \
    rm -rf /var/lib/apt/lists/*
    
# Install additional packages
RUN apt-get update && \
    apt-get install -y net-tools iproute2 gedit && \
    apt-get install -y ros-humble-plotjuggler && \
    apt-get install -y ros-humble-foxglove-bridge && \
    apt-get install -y ros-humble-turtlesim && \
    rm -rf /var/lib/apt/lists/*

# Create a non-root user named 'robotics' with home directory
RUN useradd -m -s /bin/bash robotics && \
    mkdir -p /home/robotics/data && \
    mkdir -p /home/robotics/ros2_ws/src && \
    chown -R robotics:robotics /home/robotics

# Switch to the robotics user
USER robotics
WORKDIR /home/robotics

# Source ROS2 environment for the user
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/robotics/ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]