#!/usr/bin/env bash
# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server

# Image and container naming
IMG_NAME=osrf/space-ros
# Replace `/` with `_` to comply with docker container naming
# And append `_runtime`
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")"

# Directory to map (change this to your desired host directory)
HOST_DIR="$HOME/github/starfinder/spaceros"
CONTAINER_DIR="/space-ros-workspace"

# Check if the host directory exists, if not create it
if [ ! -d "$HOST_DIR" ]; then
    echo "Creating directory $HOST_DIR"
    mkdir -p "$HOST_DIR"
fi

# Start the container
docker run --rm -it --name $CONTAINER_NAME \
    --network host \
    -e DISPLAY -e TERM \
    -e QT_X11_NO_MITSHM=1 \
    -v "$HOST_DIR:$CONTAINER_DIR" \
    $IMG_NAME

# Note: After running this script, your $HOST_DIR will be mapped to $CONTAINER_DIR in the Docker container
