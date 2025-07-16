#!/bin/bash
# SLAM Diagnostics Script
# Run this inside the Docker container to check SLAM status

echo "=== SLAM Diagnostics ==="
echo ""

echo "1. Checking if SLAM node is running..."
if ros2 node list | grep -q "lidar_slam"; then
    echo "SLAM node is running"
else
    echo "SLAM node is NOT running"
    echo "   Run: run_kitware_slam"
fi
echo ""

echo "2. Checking SLAM topics..."
SLAM_TOPICS=$(ros2 topic list | grep slam)
if [ -n "$SLAM_TOPICS" ]; then
    echo "SLAM topics found:"
    echo "$SLAM_TOPICS"
else
    echo "No SLAM topics found"
fi
echo ""

echo "3. Checking input sensor topics..."
echo "LiDAR topic (/lidar/velodyne_points):"
if ros2 topic list | grep -q "/lidar/velodyne_points"; then
    echo "LiDAR topic exists"
    echo "   Rate: $(timeout 5 ros2 topic hz /lidar/velodyne_points 2>/dev/null || echo 'No data')"
else
    echo "LiDAR topic missing"
fi

echo ""
echo "Odometry topic (/odometry/filtered):"
if ros2 topic list | grep -q "/odometry/filtered"; then
    echo "Odometry topic exists"
    echo "   Rate: $(timeout 5 ros2 topic hz /odometry/filtered 2>/dev/null || echo 'No data')"
else
    echo "Odometry topic missing"
fi
echo ""

echo "4. Checking map topic..."
if ros2 topic list | grep -q "/slam/map"; then
    echo "Map topic exists"
    echo "   Rate: $(timeout 5 ros2 topic hz /slam/map 2>/dev/null || echo 'No data')"
    
    # Get map info
    echo "   Map info:"
    timeout 10 ros2 topic echo /slam/map/info --once 2>/dev/null || echo "   Could not get map info"
else
    echo "Map topic missing"
fi
echo ""

echo "5. Checking simulation..."
if ros2 topic list | grep -q "/clock"; then
    echo "Simulation appears to be running"
else
    echo "Simulation may not be running"
    echo "   Run: run_sim"
fi
echo ""

echo "6. RViz recommendations:"
echo "   - Fixed Frame should be: odom"
echo "   - Add Map display with topic: /slam/map"
echo "   - Add PointCloud2 display with topic: /lidar/velodyne_points"
echo "   - Add Pose display with topic: /slam/pose"
echo ""

echo "=== Quick Commands ==="
echo "Start simulation: run_sim"
echo "Start SLAM: run_kitware_slam"
echo "Start RViz: run_kitware_viz"
echo "Check topics: check_topics"
echo "Monitor map: monitor_map"
echo ""
