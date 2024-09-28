Build this
https://github.com/space-ros/space-ros

Then you can use this script to run the container with this folder in it

```bash
#!/usr/bin/env bash
# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server

# Configuration
IMG_NAME="osrf/space-ros"
ROS2_DISTRO="humble"
HOST_DIR="$HOME/spaceros"
CONTAINER_DIR="/space-ros-workspace"

# Replace `/` with `_` to comply with docker container naming
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")_runtime"

# Packages to install via apt-get
APT_PACKAGES=(
    "python3-opencv"
    "ros-$ROS2_DISTRO-cv-bridge"
    "python3-pip"
    # Add more apt packages here
)

# Packages to install via pip
PIP_PACKAGES=(
    "numpy"
    # Add more pip packages here
)

# Create host directory if it doesn't exist
mkdir -p "$HOST_DIR"

# Construct installation commands
APT_INSTALL_CMD="sudo apt-get update && sudo apt-get install -y ${APT_PACKAGES[*]}"
PIP_INSTALL_CMD="pip3 install ${PIP_PACKAGES[*]}"

# Start the container
docker run --rm -it --name $CONTAINER_NAME \
    --network host \
    -e DISPLAY -e TERM \
    -e QT_X11_NO_MITSHM=1 \
    -v "$HOST_DIR:$CONTAINER_DIR" \
    $IMG_NAME \
    /bin/bash -c "
        set -e
        echo 'Installing apt packages...'
        $APT_INSTALL_CMD
        echo 'Installing pip packages...'
        $PIP_INSTALL_CMD
        echo 'Sourcing ROS 2 setup...'
        source /opt/ros/$ROS2_DISTRO/setup.bash
        echo 'Setup complete. Starting bash...'
        exec bash
    "

echo "Container exited. Host directory $HOST_DIR is mapped to $CONTAINER_DIR in the container."
# Note: After running this script, your $HOST_DIR will be mapped to $CONTAINER_DIR in the Docker container
```

Once you are in the container, cd to the workspace and run the following commands to build the workspace

```bash
colcon build --packages-select startracker_ros
```
In one terminal:
```
source install/setup.bash
ros2 run startracker_ros publisher
```

In another terminal:
```
source install/setup.bash
ros2 run startracker_ros subscriber
```
