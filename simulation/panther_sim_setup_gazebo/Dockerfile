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
    git-lfs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Add other specific dependencies for husarion_ugv_ros if known,
    # especially if they are GUI related and not covered by desktop-full.
    # For example, specific Qt libraries or other graphics libraries.
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-tf2-tools \
    # Kitware LiDAR SLAM dependencies
    libeigen3-dev \
    libpcl-dev \
    libopencv-dev \
    libnanoflann-dev \
    libg2o-dev \
    libceres-dev \
    cmake \
    build-essential \
    git \

    # MPI dependencies (required by VTK even if we disable MPI in SLAM)
    libopenmpi-dev \
    openmpi-bin \
    openmpi-common \
    && rm -rf /var/lib/apt/lists/*

# Step 4: Create a ROS 2 workspace
WORKDIR /
RUN sudo mkdir /husarion_ws
RUN git lfs clone -b main https://github.com/NSpaceTeam/husarion_ws_helper.git /husarion_ws


# Step 6: Initialize rosdep, import dependencies, install dependencies, and build the workspace
# This combines multiple steps into a single RUN command to ensure correct environment variable
# scoping and to reduce Docker image layers.
WORKDIR /husarion_ws

RUN sudo apt update && sudo apt upgrade -y

RUN rm /etc/ros/rosdep/sources.list.d/20-default.list  
RUN sudo rosdep init
RUN rosdep update --rosdistro $ROS_DISTRO
RUN rosdep install --from-paths src -y -i
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN . /opt/ros/$ROS_DISTRO/setup.bash && echo $MY_VARIABLE && colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Install system VTK packages (avoid MPI compilation issues)
# RUN apt-get update && apt-get install -y \
#     libvtk9-dev \
#     libvtk9.1 \
#     libvtk9-qt-dev \
#     libvtk9.1-qt \
#     && rm -rf /var/lib/apt/lists/*

# Use external script to build minimal Kitware LiDAR SLAM (avoiding VTK/MPI issues)
COPY ./build_kitware_slam.sh /tmp/build_kitware_slam.sh
COPY ./enhanced_3d_kitware_slam.sh /usr/local/bin/enhanced_3d_kitware_slam.sh
COPY ./slam_diagnostics.sh /usr/local/bin/slam_diagnostics.sh
COPY ./verify_slam_setup.sh /usr/local/bin/verify_slam_setup.sh
RUN chmod +x /tmp/build_kitware_slam.sh && /tmp/build_kitware_slam.sh
RUN chmod +x /usr/local/bin/enhanced_3d_kitware_slam.sh && /usr/local/bin/enhanced_3d_kitware_slam.sh
RUN chmod +x /usr/local/bin/slam_diagnostics.sh
RUN chmod +x /usr/local/bin/verify_slam_setup.sh

WORKDIR /husarion_ws
RUN echo "source /husarion_ws/install/setup.bash" >> ~/.bashrc
RUN . /husarion_ws/install/setup.bash && echo $MY_VARIABLE

# Build the workspace with minimal Kitware SLAM
RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --packages-select lidar_slam_minimal --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
RUN . /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --packages-up-to husarion_ugv --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

SHELL ["/bin/bash", "-c"]
RUN echo "source /husarion_ws/install/setup.bash" >> ~/.bashrc
RUN echo 'alias run_sim="ros2 launch husarion_ugv_gazebo simulation.launch.py"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias run_sim_marsyard="ros2 launch husarion_ugv_gazebo simulation.launch.py gz_world:=/husarion_ws/install/husarion_gz_worlds/share/husarion_gz_worlds/worlds/mars_yard.sdf"' >> ~/.bashrc && source ~/.bashrc
# Kitware SLAM aliases
RUN echo 'alias run_kitware_slam="ros2 launch lidar_slam_minimal slam.launch.py"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias run_kitware_3d="ros2 run lidar_slam_minimal lidar_slam_node_minimal"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias run_panther_kitware_slam="setup_kitware_slam && ros2 launch panther_kitware_slam panther_slam.launch.py"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias run_teleop="ros2 run teleop_twist_keyboard teleop_twist_keyboard"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias run_rviz="rviz2"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias run_kitware_viz="rviz2 -d /usr/local/share/kitware_slam_config.rviz"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias save_map="ros2 run nav2_map_server map_saver_cli -f"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_topics="ros2 topic list"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_odom="ros2 topic hz /odometry/filtered"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_scan="ros2 topic hz /lidar/velodyne_points"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_camera="ros2 topic hz /camera_front/image_raw"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_camera_back="ros2 topic hz /back_cam/zed_node/rgb/image_rect_color"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_camera_front_zed="ros2 topic hz /front_cam/zed_node/rgb/image_rect_color"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_tf="ros2 run tf2_tools view_frames.py"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias echo_scan="ros2 topic echo /lidar/velodyne_points --once"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias echo_odom="ros2 topic echo /odometry/filtered --once"' >> ~/.bashrc && source ~/.bashrc
# Kitware SLAM diagnostic aliases
RUN echo 'alias check_slam_topics="ros2 topic list | grep slam"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_slam_map="ros2 topic echo /slam/map --once"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_slam_pose="ros2 topic echo /slam/pose --once"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_map_info="ros2 topic echo /slam/map/info --once"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias monitor_map="ros2 topic hz /slam/map"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias save_kitware_map="ros2 run nav2_map_server map_saver_cli -f kitware_map --ros-args -p map_topic:=/slam/map"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias slam_diagnostics="slam_diagnostics.sh"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias verify_slam="verify_slam_setup.sh"' >> ~/.bashrc && source ~/.bashrc
# 3D SLAM specific aliases
RUN echo 'alias check_3d_map="ros2 topic hz /slam/point_cloud"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias check_local_map="ros2 topic hz /slam/local_map"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias echo_3d_map="ros2 topic echo /slam/point_cloud --once"' >> ~/.bashrc && source ~/.bashrc
RUN echo 'alias rebuild_enhanced_slam="enhanced_3d_kitware_slam.sh && cd /husarion_ws && colcon build --packages-select lidar_slam_minimal"' >> ~/.bashrc && source ~/.bashrc
RUN echo "alias setup_kitware='setup_kitware_slam && colcon build --packages-select panther_kitware_slam'" >> ~/.bashrc
# Create custom Kitware SLAM launch file for Panther
RUN cat > /usr/local/bin/setup_kitware_slam <<'KITWARE_EOF'
#!/bin/bash
# Create custom launch file for Kitware SLAM with Panther sensors
mkdir -p /husarion_ws/src/panther_kitware_slam/launch
mkdir -p /husarion_ws/src/panther_kitware_slam/config

cat > /husarion_ws/src/panther_kitware_slam/launch/panther_slam.launch.py <<'LAUNCH_EOF'
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        # Kitware LiDAR SLAM Node
        Node(
            package='lidar_slam_minimal',
            executable='lidar_slam_node_minimal',
            name='lidar_slam',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            remappings=[
                ('/pointcloud', '/lidar/velodyne_points'),
                ('/odom', '/odometry/filtered'),
            ]
        )
    ])
LAUNCH_EOF

cat > /husarion_ws/src/panther_kitware_slam/package.xml <<'PACKAGE_EOF'
<?xml version="1.0"?>
<package format="3">
  <name>panther_kitware_slam</name>
  <version>1.0.0</version>
  <description>Kitware LiDAR SLAM configuration for Husarion Panther</description>
  <maintainer email="panther@husarion.com">Panther Team</maintainer>
  <license>Apache-2.0</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <depend>rclcpp</depend>
  <depend>lidar_slam_minimal</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
PACKAGE_EOF

cat > /husarion_ws/src/panther_kitware_slam/CMakeLists.txt <<'CMAKE_EOF'
cmake_minimum_required(VERSION 3.8)
project(panther_kitware_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/\${PROJECT_NAME}/
)

ament_package()
CMAKE_EOF

echo "Kitware SLAM setup complete for Panther!"
KITWARE_EOF

RUN chmod +x /usr/local/bin/setup_kitware_slam

# Create specialized RViz configuration for 3D Kitware LiDAR SLAM
RUN cat > /usr/local/share/kitware_slam_config.rviz <<'KITWARE_RVIZ_EOF'
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /3D SLAM Map1
        - /Local 3D Map1
        - /Raw Lidar1
        - /2D Map1
        - /Robot Model1
        - /TF1
      Splitter Ratio: 0.5
    Tree Height: 557
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Name: Grid
      Value: true
    - Alpha: 0.8
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: 2D Map
      Topic:
        Value: /slam/map
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Enabled: true
      Name: Raw Lidar
      Topic:
        Value: /lidar/velodyne_points
      Value: true
    - Alpha: 0.8
      Class: rviz_default_plugins/PointCloud2
      Color: 0; 255; 0
      Color Transformer: Z
      Enabled: true
      Name: 3D SLAM Map
      Topic:
        Value: /slam/point_cloud
      Value: true
      Size (m): 0.03
      Style: Points
    - Alpha: 1.0
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 0
      Color Transformer: Intensity
      Enabled: true
      Name: Local 3D Map
      Topic:
        Value: /slam/local_map
      Value: true
      Size (m): 0.02
      Style: Points
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 0; 255
      Color Transformer: Z
      Enabled: false
      Name: Registered Points
      Topic:
        Value: /slam/registered_points
      Value: false
      Size (m): 0.02
      Style: Points
    - Alpha: 1
      Class: rviz_default_plugins/Path
      Color: 25; 255; 0
      Enabled: true
      Name: Trajectory
      Topic:
        Value: /slam/trajectory
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: false
      Name: Loop Closures
      Topic:
        Value: /slam/loop_closures
      Value: false
    - Class: rviz_default_plugins/Image
      Enabled: false
      Name: Camera Front
      Topic:
        Value: /front_cam/zed_node/rgb/image_rect_color
      Value: false
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Enabled: true
      Name: Robot Model
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
    - Class: rviz_default_plugins/SetInitialPose
      Theta std deviation: 0.2617993877991494
      Topic: /initialpose
      X std deviation: 0.5
      Y std deviation: 0.5
    - Class: rviz_default_plugins/SetGoal
      Topic: /goal_pose
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 25.0
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1.0
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.4
      Target Frame: <Fixed Frame>
      Yaw: 0.8
    Saved: ~
KITWARE_RVIZ_EOF

# Step 7: Set up the entrypoint to source ROS 2 and the workspace
# Ensure entrypoint.sh is present in the same directory as the Dockerfile (the build context).
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Step 8: Default command
# This provides an interactive bash shell when the container starts without a command.
CMD ["bash"]
