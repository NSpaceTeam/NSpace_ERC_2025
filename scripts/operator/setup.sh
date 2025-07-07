#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
IMAGE_NAME="my-husarion-operator:jazzy" # Ili vaše željeno ime slike:tag
DOCKERFILE_DIR="."                     # Pretpostavlja se da je Dockerfile u trenutnom direktorijumu
CPU_LIMIT="6"                          # Broj CPU jezgara za kontejner
MEMORY_LIMIT="12g"                     # RAM za kontejner
BASHRC_FILE="$HOME/.bashrc"
# --- Izmene na osnovu vašeg zahteva ---
CONTAINER_NAME="operator"              # Ime koje će kontejner imati dok radi
ALIAS_NAME="run_operator"              # Komanda (alias) za pokretanje kontejnera
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

# 2. Add a unified alias to .bashrc
echo ""
echo "Step 2: Configuring the alias '$ALIAS_NAME' in $BASHRC_FILE..."

# Helper function to add/update alias
add_or_update_alias() {
    local alias_name="$1"
    local alias_command="$2"
    local alias_comment="$3"

    # Remove existing alias definition if present to avoid duplicates
    if grep -q "# ${alias_comment}" "$BASHRC_FILE"; then
        sed -i "/# ${alias_comment}/d" "$BASHRC_FILE"
        sed -i "/alias ${alias_name}=/d" "$BASHRC_FILE"
        echo "Removed existing alias definition for '${alias_name}' to update it."
    fi

    echo "" >> "$BASHRC_FILE"
    echo "# ${alias_comment}" >> "$BASHRC_FILE"
    echo "alias ${alias_name}='${alias_command}'" >> "$BASHRC_FILE"
    echo "Added/Updated alias '${alias_name}' in $BASHRC_FILE."
}

# The command to execute before docker run for X11 access
XHOST_CMD="xhost +local:docker &&"

# Unified docker run command.
# --rm : Automatically remove the container when it exits. Prevents name conflicts.
# --name ${CONTAINER_NAME} : Sets the name of the container.
# --gpus all : Modern way to enable GPU access for NVIDIA and often AMD/Intel.
# --device /dev/dri : Provides access to Direct Rendering Infrastructure, good for compatibility.
UNIFIED_DOCKER_CMD="${XHOST_CMD} docker run -it --rm \
  --name ${CONTAINER_NAME} \
  --gpus all \
  --network host \
  --privileged \
  --env=\"DISPLAY\" \
  --env=\"QT_X11_NO_MITSHM=1\" \
  --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" \
  --volume=\"/var/lib/husarnet:/var/lib/husarnet\" \
  --device /dev/dri \
  --memory=${MEMORY_LIMIT} \
  --cpus=${CPU_LIMIT} \
  ${IMAGE_NAME} bash"

# Add the single, unified alias
add_or_update_alias "$ALIAS_NAME" "$UNIFIED_DOCKER_CMD" "Operator Docker: Run container '${CONTAINER_NAME}' with GUI/GPU"

echo ""
echo "--- Setup Complete ---"
echo "Docker image '$IMAGE_NAME' is built."
echo "Alias '$ALIAS_NAME' has been configured in your $BASHRC_FILE."
echo ""
echo "IMPORTANT NEXT STEPS:"
echo "1. To apply the changes, source your .bashrc file or open a new terminal:"
echo "   source $BASHRC_FILE"
echo ""
echo "2. You can now start the container using the command:"
echo "   $ALIAS_NAME"
echo ""
echo "When you run this command, a container named '${CONTAINER_NAME}' will be started."
