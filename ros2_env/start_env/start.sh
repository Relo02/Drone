#!/bin/bash

CONTAINER_NAME="ros_humble_container"
IMAGE_NAME="lorenzo195815/ros:humble-desktop"
VNC_PASSWORD="password"
DISPLAY_SIZE="1920x1080"

docker rm -f ${CONTAINER_NAME} >/dev/null 2>&1 || true
mkdir -p ../data ../ros2_ws
xhost +local:root >/dev/null 2>&1

docker run -it \
    --platform linux/amd64 \
    -p 8080:8080 \
    -p 5902:5902 \
    -p 8765:8765 \
    --env="DISPLAY=:2" \
    --env="VNC_PASSWORD=${VNC_PASSWORD}" \
    --volume="$PWD/../data:/home/ros/data" \
    --volume="$PWD/../ros2_ws:/home/ros/ros2_ws" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --cap-add=SYS_PTRACE \
    --security-opt seccomp=unconfined \
    --name "$CONTAINER_NAME" \
    "$IMAGE_NAME" \
    bash -c "\
        rm -rf /tmp/.X11-unix/X* && \
        mkdir -p ~/.vnc && \
        echo '${VNC_PASSWORD}' | vncpasswd -f > ~/.vnc/passwd && \
        chmod 600 ~/.vnc/passwd && \
        vncserver :2 -geometry ${DISPLAY_SIZE} -depth 24 && \
        export DISPLAY=:2 && \
        /usr/share/novnc/utils/launch.sh --listen 8080 --vnc localhost:5902 & \
        bash"