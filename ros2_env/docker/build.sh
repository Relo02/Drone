#!/bin/bash

# Specify the image name
IMAGE_NAME="lorenzo195815/quad"

# Build the Docker image for the correct platform
docker build --platform linux/amd64 -t $IMAGE_NAME .

# Push the image to Docker Hub (optional)
docker push $IMAGE_NAME