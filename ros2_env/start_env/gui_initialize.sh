#!/bin/bash

# Configuration
ROS_CONTAINER_NAME="ros_humble_container"
NOVNC_CONTAINER_NAME="ros_novnc"
VNC_PASSWORD="password"
DISPLAY_SIZE="1920x1080"
PLATFORM="linux/arm64"  # Changed for Apple Silicon

# Ensure ROS container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${ROS_CONTAINER_NAME}$"; then
    echo "Starting ROS container..."
    docker run -d --rm \
        --platform ${PLATFORM} \
        --name ${ROS_CONTAINER_NAME} \
        -v $(pwd)/../data:/home/ros/data \
        -v $(pwd)/../ros2_ws:/home/ros/ros2_ws \
        lorenzo195815/ros:humble-desktop \
        sleep infinity
fi

# Wait for container to be fully up
sleep 2

# Start VNC server
echo "Setting up VNC..."
docker exec ${ROS_CONTAINER_NAME} bash -c "\
    mkdir -p ~/.vnc && \
    echo '${VNC_PASSWORD}' | vncpasswd -f > ~/.vnc/passwd && \
    chmod 600 ~/.vnc/passwd && \
    vncserver :1 -geometry ${DISPLAY_SIZE} -depth 24 && \
    websockify --web /usr/share/novnc/ 6080 localhost:5901"

# Start noVNC
echo "Starting noVNC..."
docker run -d --rm \
    --name ${NOVNC_CONTAINER_NAME} \
    --network container:${ROS_CONTAINER_NAME} \
    -e DISPLAY_WIDTH=${DISPLAY_SIZE%x*} \
    -e DISPLAY_HEIGHT=${DISPLAY_SIZE#*x} \
    theasp/novnc