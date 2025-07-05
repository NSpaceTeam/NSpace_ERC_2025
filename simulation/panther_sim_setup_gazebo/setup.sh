#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
IMAGE_NAME="my-husarion-app:jazzy" # Or your preferred image name:tag
DOCKERFILE_DIR="." # Assumes Dockerfile is in the current directory
CPU_LIMIT="4" # Default CPU cores for the container
MEMORY_LIMIT="8g" # Default RAM for the container
BASHRC_FILE="$HOME/.bashrc"
# --- End Configuration ---

echo "--- Husarion Docker Setup Script ---"

# 1. Build the Docker image
echo ""
echo "Step 1: Building Docker image '$IMAGE_NAME'..."
if docker build -t "$IMAGE_NAME" "$DOCKERFILE_DIR"; then
    echo "Docker image '$IMAGE_NAME' built successfully."
else
    echo "ERROR: Docker image build failed."
    exit 1
fi

# 2. Add aliases to .bashrc
echo ""
echo "Step 2: Configuring aliases in $BASHRC_FILE..."

# Common alias prefix
ALIAS_PREFIX="run_husarion"

# Helper function to add/update alias
add_or_update_alias() {
    local alias_name="$1"
    local alias_command="$2"
    local alias_comment="$3"

    # Remove existing alias definition if present
    if grep -q "# ${alias_comment}" "$BASHRC_FILE"; then
        # More robust way to remove multi-line alias if we had one,
        # but for single line, this should suffice.
        # We'll remove the comment line and the alias line.
        sed -i "/# ${alias_comment}/d" "$BASHRC_FILE"
        sed -i "/alias ${alias_name}=/d" "$BASHRC_FILE"
        echo "Removed existing alias definition for '${alias_name}' to update it."
    fi

    echo "" >> "$BASHRC_FILE"
    echo "# ${alias_comment}" >> "$BASHRC_FILE"
    echo "alias ${alias_name}='${alias_command}'" >> "$BASHRC_FILE"
    echo "Added/Updated alias '${alias_name}' in $BASHRC_FILE."
}

# Base docker run options for GUI
BASE_DOCKER_RUN_OPTIONS="-it --rm \
    --cpus=\"${CPU_LIMIT}\" \
    -m \"${MEMORY_LIMIT}\" \
    --env=\"DISPLAY\" \
    --env=\"QT_X11_NO_MITSHM=1\" \
    --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" \
    -v /var/lib/husarnet:/var/lib/husarnet \ "

# Command to execute before docker run for X11 access
XHOST_CMD="xhost +local:docker &&"

# Alias for No GPU
ALIAS_NAME_NO_GPU="${ALIAS_PREFIX}_nogpu"
ALIAS_CMD_NO_GPU="${XHOST_CMD} docker run ${BASE_DOCKER_RUN_OPTIONS} \
    --name husarion_nogpu \
    ${IMAGE_NAME} bash"
add_or_update_alias "$ALIAS_NAME_NO_GPU" "$ALIAS_CMD_NO_GPU" "Husarion Docker: Run without dedicated GPU acceleration"

# Alias for NVIDIA GPU
ALIAS_NAME_NVIDIA="${ALIAS_PREFIX}_nvidia"
ALIAS_CMD_NVIDIA="${XHOST_CMD} docker run ${BASE_DOCKER_RUN_OPTIONS} \
    --gpus all \
    --name husarion_nvidia \
    ${IMAGE_NAME} bash"
add_or_update_alias "$ALIAS_NAME_NVIDIA" "$ALIAS_CMD_NVIDIA" "Husarion Docker: Run with NVIDIA GPU acceleration"

# Alias for INTEL INTEGRATED GPU
ALIAS_NAME_INTEL="${ALIAS_PREFIX}_intel"
ALIAS_CMD_INTEL="${XHOST_CMD} docker run ${BASE_DOCKER_RUN_OPTIONS} \
    --ipc=host \
    --device=/dev/dri:/dev/dri \
    --name husarion_intel \
    ${IMAGE_NAME} bash"
add_or_update_alias "$ALIAS_NAME_INTEL" "$ALIAS_CMD_INTEL" "Husarion Docker: Run with INTEL support"

# Alias for AMD GPU
# Try to get video group GID, fallback to render group GID if video group not found or getent fails
VIDEO_GROUP_GID=$(getent group video | cut -d: -f3)
if [ -z "$VIDEO_GROUP_GID" ]; then
    echo "Warning: 'video' group not found. Trying 'render' group for AMD GPU."
    RENDER_GROUP_GID=$(getent group render | cut -d: -f3)
    if [ -z "$RENDER_GROUP_GID" ]; then
        echo "ERROR: Neither 'video' nor 'render' group found. AMD GPU alias might not have correct permissions."
        AMD_GROUP_ADD_CMD="" # leave empty if no group found
    else
        AMD_GROUP_ADD_CMD="--group-add=${RENDER_GROUP_GID}"
    fi
else
    AMD_GROUP_ADD_CMD="--group-add=${VIDEO_GROUP_GID}"
fi

ALIAS_NAME_AMD="${ALIAS_PREFIX}_amd"
ALIAS_CMD_AMD="${XHOST_CMD} docker run ${BASE_DOCKER_RUN_OPTIONS} \
    --device=/dev/dri:/dev/dri \
    ${AMD_GROUP_ADD_CMD} \
    --name husarion_amd \
    ${IMAGE_NAME} bash"
add_or_update_alias "$ALIAS_NAME_AMD" "$ALIAS_CMD_AMD" "Husarion Docker: Run with AMD GPU acceleration"

echo ""
echo "--- Setup Complete ---"
echo "Docker image '$IMAGE_NAME' is built."
echo "Aliases have been added/updated in your $BASHRC_FILE."
echo ""
echo "IMPORTANT NEXT STEPS:"
echo "1. Source your .bashrc file or open a new terminal:"
echo "   source $BASHRC_FILE"
echo ""
echo "The 'xhost +local:docker' command will now run automatically when you use the aliases."
echo "You can use aliases like '$ALIAS_NAME_NO_GPU', '$ALIAS_NAME_NVIDIA', '$ALIAS_NAME_AMD', or '$ALIAS_NAME_INTEL' to start the container."
echo "Example: run_husarion_amd"
