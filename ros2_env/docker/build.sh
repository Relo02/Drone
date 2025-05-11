#!/bin/bash

# Exit on error and undefined variables
set -eu

# Specify the image name and tag
IMAGE_NAME="lorenzo195815/ros"
TAG="humble-desktop"

# Build the Docker image for the correct platform
echo "Building Docker image ${IMAGE_NAME}:${TAG}..."
docker build --platform linux/amd64 -t ${IMAGE_NAME}:${TAG} .

# Push the image to Docker Hub (optional)
read -p "Push image to Docker Hub? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    docker push ${IMAGE_NAME}:${TAG}
fi