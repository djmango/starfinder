Build this
https://github.com/space-ros/space-ros

Then you can use this script to run the container with this folder in it

```bash
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
HOST_DIR="$HOME/spaceros"
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
```

Once you are in the container, cd to the workspace and run the following commands to build the workspace

```bash
colcon build --packages-select startracker_ros
```
In one terminal:
```
source install/setup.bash
ros2 run startracker_ros starfinder_publisher
```

In another terminal:
```
source install/setup.bash
ros2 run startracker_ros starfinder_subscriber
```
