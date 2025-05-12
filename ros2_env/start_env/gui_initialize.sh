#!/bin/bash

# Configuration
ROS_CONTAINER_NAME="ros_humble_container"
NOVNC_CONTAINER_NAME="ros_novnc"
DISPLAY_SIZE="1920x1080"
PLATFORM="linux/amd64"

# Ensure ROS container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${ROS_CONTAINER_NAME}$"; then
    echo "ROS container is not running. Please start it first with ./start.sh"
    exit 1
fi

# Start noVNC container (links into existing ROS container's network)
echo "Starting noVNC on http://localhost:8080..."
docker run -d --rm \
    --platform ${PLATFORM} \
    --name ${NOVNC_CONTAINER_NAME} \
    --network container:${ROS_CONTAINER_NAME} \
    theasp/novnc

echo "noVNC running. Open http://localhost:8080 in your browser."