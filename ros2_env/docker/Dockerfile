FROM osrf/ros:humble-desktop

# Set the environment variable to non-interactive
ENV DEBIAN_FRONTEND=noninteractive

# Switch to root user to install packages
USER root

# Install necessary packages including VNC, GUI tools, Gazebo, OpenGL libs
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    nano \
    net-tools \
    tmux \
    iputils-ping \
    iproute2 \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    gedit \
    x11-utils \
    xvfb \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    tigervnc-standalone-server \
    tigervnc-common \
    websockify \
    novnc \
    ros-humble-desktop-full \
    ros-humble-plotjuggler* \
    ros-humble-foxglove-bridge \
    ros-humble-turtlesim* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    wget \
    lsb-release \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Set up environment variables for GUI apps
ENV DISPLAY=:1
ENV LIBGL_ALWAYS_INDIRECT=1
# Add these to your existing ENV section
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/gazebo_plugins/models
ENV GAZEBO_RESOURCE_PATH=/opt/ros/humble/share/gazebo_plugins/worlds
ENV SDL_AUDIODRIVER=dummy

# Create a non-root user named 'ros' with home directory
RUN id -u ros || useradd -m -s /bin/bash ros && \
    mkdir -p /home/ros/data && \
    mkdir -p /home/ros/ros2_ws/src && \
    chown -R ros:ros /home/ros

# Switch to the ros user
USER ros
WORKDIR /home/ros

# Set up VNC password (change 'password' to your desired password)
RUN mkdir -p ~/.vnc && echo "password" | vncpasswd -f > ~/.vnc/passwd && chmod 600 ~/.vnc/passwd

# Source ROS environment for the user and set useful aliases
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros/ros2_ws/install/setup.bash || true" >> ~/.bashrc && \
    echo 'alias ls="ls --color=auto"' >> ~/.bashrc && \
    echo 'alias ll="ls -la"' >> ~/.bashrc

# Default command
CMD ["bash"]