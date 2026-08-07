#!/bin/bash

# Directories & Variables
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Load common variables (auto-copy from example if not exists)
if [ ! -f "$SCRIPT_DIR/config.sh" ]; then
    cp "$SCRIPT_DIR/config.sh.example" "$SCRIPT_DIR/config.sh"
fi
source "$SCRIPT_DIR/config.sh"

# Check if the image exists
if ! docker image inspect "$IMAGE_NAME" > /dev/null 2>&1; then
    echo "Error: Image $IMAGE_NAME not found."
    echo "Build locally first: ./build.sh"
    exit 1
fi

# Pre-create build artifact dirs so Docker does not create them as root
mkdir -p "$ROS2_WS_ROOT/build" "$ROS2_WS_ROOT/install" "$ROS2_WS_ROOT/log"

# [1/2] Enable X11 access for Docker
echo "==> [1/2] Enabling X11 access for Docker (xhost +local:docker)..."
xhost +local:docker > /dev/null 2>&1

# [2/2] Run the Docker container
echo "==> [2/2] Checking container '$CONTAINER_NAME'..."

# Reuse existing container if present; otherwise create a new one
if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}$"; then

    # Start the container if it is stopped
    if [ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" = "false" ]; then
        echo "--> Container exists but is stopped. Starting '$CONTAINER_NAME'..."
        docker start "$CONTAINER_NAME" > /dev/null
    fi

    # Attach a new shell to the running container
    echo "--> Attaching to running container '$CONTAINER_NAME'..."
    echo "---------- container output ----------"
    echo
    docker exec -it \
        -e HOST_UID="$(id -u)" \
        -e HOST_GID="$(id -g)" \
        "$CONTAINER_NAME" \
        /entrypoint.sh /bin/bash

else
    # No existing container: create and run a new one
    echo "--> Container '$CONTAINER_NAME' not found. Creating a new one..."
    echo "---------- container output ----------"
    echo
    docker run -it --rm \
        --name "$CONTAINER_NAME" \
        --privileged \
        --network host \
        --ipc=host \
        \
        -e DISPLAY="$DISPLAY" \
        -e QT_X11_NO_MITSHM=1 \
        -e HOST_UID="$(id -u)" \
        -e HOST_GID="$(id -g)" \
        -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        -e XAUTHORITY=/root/.Xauthority \
        \
        -v "$ROS2_WS_ROOT:/ros2_ws" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /etc/localtime:/etc/localtime:ro \
        -v "$XAUTHORITY_PATH":/root/.Xauthority:rw \
        \
        "$IMAGE_NAME"
fi
