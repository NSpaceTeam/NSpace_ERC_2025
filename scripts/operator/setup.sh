#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# --- Configuration ---
IMAGE_NAME="my-husarion-operator:jazzy" # Ili vaše željeno ime slike:tag
DOCKERFILE_DIR="."                     # Pretpostavlja se da je Dockerfile u trenutnom direktorijumu
CPU_LIMIT="4"                          # Broj CPU jezgara za kontejner
MEMORY_LIMIT="8g"                     # RAM za kontejner
BASHRC_FILE="$HOME/.bashrc"
# --- Konfiguracija za jedinstveni kontejner ---
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

    # Uklanja postojeću definiciju aliasa kako bi se izbegli duplikati
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

# Komanda koja se izvršava pre 'docker run' za pristup X11 (GUI)
XHOST_CMD="xhost +local:docker &&"

# Univerzalna 'docker run' komanda koja kombinuje opcije za različite GPU-ove.
# VAŽNO: Opcija --rm je namerno izostavljena kako se kontejner ne bi brisao nakon izlaska.
# --device /dev/dri : Ključno za Intel i AMD grafiku.
# --privileged : Daje proširene privilegije kontejneru, što često rešava probleme sa pristupom hardveru.
UNIFIED_DOCKER_CMD="${XHOST_CMD} docker run -it \
  --name ${CONTAINER_NAME} \
  --network host \
  --privileged \
  --env=\"DISPLAY\" \
  --env=\"QT_X11_NO_MITSHM=1\" \
  --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" \
  -v /var/lib/husarnet:/var/lib/husarnet \
  --device=/dev/dri:/dev/dri \
  --memory=${MEMORY_LIMIT} \
  --cpus=${CPU_LIMIT} \
  ${IMAGE_NAME} bash"

# Dodaje jedan, jedinstveni alias u .bashrc
add_or_update_alias "$ALIAS_NAME" "$UNIFIED_DOCKER_CMD" "Operator Docker: Run container '${CONTAINER_NAME}' with universal GUI/GPU support"

echo ""
echo "--- Setup Complete ---"
echo "Docker image '$IMAGE_NAME' is built."
echo "Alias '$ALIAS_NAME' has been configured in your $BASHRC_FILE."
echo ""
echo "IMPORTANT NEXT STEPS:"
echo "1. To apply the changes, close this terminal and open a new one, OR run:"
echo "   source $BASHRC_FILE"
echo ""
echo "2. You can now start the container using the single command:"
echo "   $ALIAS_NAME"
echo ""
echo "The container will be named '${CONTAINER_NAME}' and will NOT be deleted when you exit."
echo "To run it again, you will first need to remove the old one with 'docker rm ${CONTAINER_NAME}'."
