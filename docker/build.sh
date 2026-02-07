#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set image name
IMAGE_NAME="robot-kinematics-sandbox-humble:latest"

# Build the Docker image
echo "Building Docker image: $IMAGE_NAME..."
docker build -t $IMAGE_NAME -f "$SCRIPT_DIR/Dockerfile" "$SCRIPT_DIR"

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "Docker image $IMAGE_NAME built successfully!"
else
    echo "Failed to build Docker image. Check the logs above for errors."
    exit 1
fi