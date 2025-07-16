#!/bin/bash
# Verify that Enhanced 3D Kitware SLAM is properly set up

echo "=== Enhanced 3D Kitware SLAM Setup Verification ==="
echo

# Check if the enhanced SLAM script is available
if [ -f "/usr/local/bin/enhanced_3d_kitware_slam.sh" ]; then
    echo "✓ Enhanced SLAM script found at /usr/local/bin/enhanced_3d_kitware_slam.sh"
else
    echo "✗ Enhanced SLAM script NOT found!"
    echo "  Expected location: /usr/local/bin/enhanced_3d_kitware_slam.sh"
    echo "  You may need to rebuild the Docker image or copy the script manually."
    echo
fi

# Check if the minimal_slam_node.cpp exists
if [ -f "/husarion_ws/src/lidar_slam_minimal/src/minimal_slam_node.cpp" ]; then
    echo "✓ Enhanced SLAM node source code found"
    
    # Check if it contains the Enhanced3DSlamNode class
    if grep -q "Enhanced3DSlamNode" "/husarion_ws/src/lidar_slam_minimal/src/minimal_slam_node.cpp"; then
        echo "✓ Enhanced3DSlamNode class found in source code"
    else
        echo "✗ Enhanced3DSlamNode class NOT found in source code"
        echo "  The SLAM node may not be properly enhanced."
    fi
else
    echo "✗ Enhanced SLAM node source code NOT found!"
    echo "  Expected location: /husarion_ws/src/lidar_slam_minimal/src/minimal_slam_node.cpp"
fi

# Check if the lidar_slam_minimal package is built
if [ -f "/husarion_ws/install/lidar_slam_minimal/lib/lidar_slam_minimal/lidar_slam_node_minimal" ]; then
    echo "✓ Enhanced SLAM executable built successfully"
else
    echo "✗ Enhanced SLAM executable NOT found!"
    echo "  Expected location: /husarion_ws/install/lidar_slam_minimal/lib/lidar_slam_minimal/lidar_slam_node_minimal"
    echo "  You may need to build the package: cd /husarion_ws && colcon build --packages-select lidar_slam_minimal"
fi

echo
echo "=== ROS2 Environment Check ==="

# Source the ROS2 environment
source /opt/ros/jazzy/setup.bash 2>/dev/null || echo "Warning: Could not source ROS2 jazzy"
source /husarion_ws/install/setup.bash 2>/dev/null || echo "Warning: Could not source husarion_ws"

# Check if the package is available
if ros2 pkg list | grep -q "lidar_slam_minimal"; then
    echo "✓ lidar_slam_minimal package is available in ROS2"
else
    echo "✗ lidar_slam_minimal package NOT available in ROS2"
    echo "  You may need to source the workspace: source /husarion_ws/install/setup.bash"
fi

# Check if the executable is available
if ros2 pkg executables lidar_slam_minimal | grep -q "lidar_slam_node_minimal"; then
    echo "✓ lidar_slam_node_minimal executable is available"
else
    echo "✗ lidar_slam_node_minimal executable NOT available"
fi

echo
echo "=== Quick Fix Instructions ==="
echo "If any checks failed, try these commands:"
echo "1. Regenerate the enhanced SLAM node:"
echo "   enhanced_3d_kitware_slam.sh"
echo "2. Rebuild the package:"
echo "   cd /husarion_ws && colcon build --packages-select lidar_slam_minimal"
echo "3. Source the workspace:"
echo "   source /husarion_ws/install/setup.bash"
echo "4. Test the SLAM node:"
echo "   ros2 run lidar_slam_minimal lidar_slam_node_minimal"
echo
echo "=== Useful Aliases ==="
echo "rebuild_enhanced_slam  - Regenerate and rebuild the enhanced SLAM"
echo "check_3d_map          - Check if /slam/point_cloud topic is publishing"
echo "echo_3d_map           - Show a sample point cloud message"
echo "slam_diagnostics      - Run full SLAM diagnostics"
