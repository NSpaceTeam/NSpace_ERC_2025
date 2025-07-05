# Step 1: Start from an official ROS 2 Jazzy desktop-full image
FROM osrf/ros:jazzy-desktop-full

# install ros2 packages

    
# Alternatively, ros:jazzy-perception might be sufficient if you don't need full desktop tools
# but osrf/ros:jazzy-desktop-full is more comprehensive for GUIs.

# Set the default shell to bash for all subsequent RUN, CMD, ENTRYPOINT commands
SHELL ["/bin/bash", "-c"]

# Step 2: Set environment variables
# DEBIAN_FRONTEND=noninteractive prevents interactive prompts during apt-get installs.
ENV DEBIAN_FRONTEND=noninteractive
# Set ROS_DISTRO, though it's usually inherited from the base image, explicitly setting it can be safer.
ENV ROS_DISTRO=jazzy
# Set HUSARION_ROS_BUILD_TYPE, so it's available globally for all subsequent RUN commands and at runtime.
ENV HUSARION_ROS_BUILD_TYPE=simulation

# Step 3: Install necessary system dependencies and tools
# Many GUI libraries will be included in desktop-full.
# python3-vcstool is needed for the 'vcs import' command.
RUN sudo apt-get update && sudo apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Add other specific dependencies for husarion_ugv_ros if known,
    # especially if they are GUI related and not covered by desktop-full.
    # For example, specific Qt libraries or other graphics libraries.
    && rm -rf /var/lib/apt/lists/*

# Step 4: Create workspace
WORKDIR /
RUN sudo mkdir /operator_ws
RUN sudo mkdir /operator_ws/scripts

RUN curl -s https://install.husarnet.com/install.sh | sudo bash

# Postavljamo environment varijable koje će biti dostupne u kontejneru i instalacija Husarnet-a i DDS-a

ENV RELEASE="v1.3.5"
ENV ARCH="amd64"

RUN curl -L https://github.com/husarnet/husarnet-dds/releases/download/$RELEASE/husarnet-dds-linux-$ARCH -o /usr/local/bin/husarnet-dds
RUN chmod +x /usr/local/bin/husarnet-dds

RUN sudo apt-get update
RUN sudo apt-get install -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp 
RUN apt-get install -y nano
RUN apt-get install -y gedit

    
#Dodatna podesavanja

RUN git clone -b main https://github.com/Micoo1512/config.git /operator_ws/scripts
   
RUN mv /operator_ws/scripts/cyclonedds.xml /var/tmp/

# Step 6: Initialize rosdep, import dependencies, install dependencies, and build the workspace
# This combines multiple steps into a single RUN command to ensure correct environment variable
# scoping and to reduce Docker image layers.
WORKDIR /operator_ws

# Step 7: Set up the entrypoint to source ROS 2 and the workspace
# Ensure entrypoint.sh is present in the same directory as the Dockerfile (the build context).
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
SHELL ["/bin/bash", "-c"]
# Step 7: Set up the entrypoint to source ROS 2 and the workspace
# Ensure entrypoint.sh is present in the same directory as the Dockerfile (the build context).
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Step 8: Default command
# This provides an interactive bash shell when the container starts without a command.
CMD ["bash"]
