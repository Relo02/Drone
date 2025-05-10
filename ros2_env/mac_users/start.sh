#!/bin/bash

# Specify the container name and image
CONTAINER_NAME="ros_humble_container"
IMAGE_NAME="osrf/ros:humble-desktop"

# Pull the latest ROS Humble image for the correct platform
echo "Pulling the latest ROS Humble image: $IMAGE_NAME..."
docker pull --platform linux/amd64 $IMAGE_NAME

# Check if the container exists
if docker ps -a --format "{{.Names}}" | grep -q "^$CONTAINER_NAME$"; then
    echo "Container $CONTAINER_NAME exists."

    # Check if the container is running
    if [ "$(docker inspect -f "{{.State.Running}}" $CONTAINER_NAME 2>/dev/null)" == "true" ]; then
        echo "Container $CONTAINER_NAME is running. Stopping it now..."
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    else
        echo "Container $CONTAINER_NAME is not running."
        docker rm $CONTAINER_NAME
    fi
else
    echo "Container $CONTAINER_NAME does not exist."
fi

# Ensure the local 'data' and 'ros2_ws' folders exist
PWD_DIR=$(pwd)
DATA_FOLDER="$PWD_DIR/../data"
ROS2_WS_FOLDER="$PWD_DIR/../ros2_ws"

mkdir -p "$DATA_FOLDER"
mkdir -p "$ROS2_WS_FOLDER"

# Run the ROS Humble container with the correct platform
docker run -it \
    --platform linux/amd64 \
    --user "$(id -u):$(id -g)" \
    --env="DISPLAY=novnc:0.0" \
    --env="QT_X11_NO_MITSHM=1" \
    --net=host \
    --rm \
    --volume="$DATA_FOLDER:/home/ros/data" \
    --volume="$ROS2_WS_FOLDER:/home/ros/ros2_ws" \
    --name "$CONTAINER_NAME" \
    -w /home/ros \
    "$IMAGE_NAME" \
    bash