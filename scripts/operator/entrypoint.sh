#!/bin/bash
set -e

# Source ROS 2 environment
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  echo "Sourcing ROS environment: /opt/ros/${ROS_DISTRO}/setup.bash"
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace environment
if [ -f "/operator_ws/install/setup.bash" ]; then
  echo "Sourcing workspace environment: /ros2_ws/install/setup.bash"
  source "/operator_ws/install/setup.bash"
fi

echo "Executing command: $@"
# Execute the command passed to docker run (or the CMD if no command is passed)
exec "$@"
