#!/bin/bash

CONTAINER_NAME="ros_humble_container"
IMAGE_NAME="lorenzo195815/ros:humble-desktop"
VNC_PASSWORD="password"  # Change if needed
DISPLAY_SIZE="1920x1080"

# Clean up existing container
if docker ps -a --format "{{.Names}}" | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing container ${CONTAINER_NAME}..."
    docker stop ${CONTAINER_NAME} >/dev/null 2>&1 || true
    docker rm ${CONTAINER_NAME} >/dev/null 2>&1 || true
fi

# Create folders
mkdir -p ../data ../ros2_ws

# Run with VNC
docker run -it \
    --platform linux/amd64 \
    -p 6080:6080 \
    -p 5901:5901 \
    --env="DISPLAY=:1" \
    --env="VNC_PASSWORD=${VNC_PASSWORD}" \
    --volume="$PWD/../data:/home/ros/data" \
    --volume="$PWD/../ros2_ws:/home/ros/ros2_ws" \
    --name "$CONTAINER_NAME" \
    --workdir /home/ros \
    "$IMAGE_NAME" \
    bash -c "\
        mkdir -p ~/.vnc && \
        echo '$VNC_PASSWORD' | vncpasswd -f > ~/.vnc/passwd && \
        chmod 600 ~/.vnc/passwd && \
        vncserver :1 -geometry $DISPLAY_SIZE -depth 24 && \
        websockify --web /usr/share/novnc/ 6080 localhost:5901 && \
        bash"