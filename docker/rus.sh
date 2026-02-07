#!/bin/bash

# Get workspace root directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Set image name
IMAGE_NAME="robot-kinematics-sandbox-humble:latest"

# Set container name
CONTAINER_NAME="robot-kinematics-sandbox-humble"

# Check if the image exists
if ! docker image inspect $IMAGE_NAME > /dev/null 2>&1; then
    echo "Error: Image $IMAGE_NAME not found. Please run ./build.sh first."
    exit 1
fi

# Enable X11 access for Docker
echo "Enabling X11 access for Docker..."
xhost +local:docker

# Run the Docker container
echo "Running Docker container from image: $IMAGE_NAME..."
docker run -it --rm \
    --privileged \
    --network host \
    --ipc=host \
    -e ROS_DOMAIN_ID=98 \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $ROS2_WS_ROOT:/ros2_ws \
    --name $CONTAINER_NAME \
    $IMAGE_NAME

# Disable X11 access after container exit
echo "Disabling X11 access after container exit..."
xhost -local:docker