#!/bin/bash
# =============================================================================
# Taggy - ROS2 Foxy System Dependencies Installation
# =============================================================================
# This script installs ROS2 packages and system dependencies via apt
# Run with: ./install_ros2_deps.sh
# =============================================================================

set -e

echo "=============================================="
echo "  Taggy - ROS2 Foxy Dependencies Installer"
echo "=============================================="
echo ""

# Check if running as root
if [[ $EUID -eq 0 ]]; then
    echo "Warning: Running as root. This is usually not recommended."
fi

# Check ROS2 Foxy installation
if [ ! -f "/opt/ros/foxy/setup.bash" ]; then
    echo "ERROR: ROS2 Foxy not found at /opt/ros/foxy"
    echo "Please install ROS2 Foxy first:"
    echo "  sudo apt install ros-foxy-desktop"
    echo "Or follow: https://docs.ros.org/en/foxy/Installation.html"
    exit 1
fi

echo "[1/5] Updating package lists..."
sudo apt update

echo ""
echo "[2/5] Installing ROS2 Foxy Navigation Stack (Nav2)..."
sudo apt install -y \
    ros-foxy-nav2-bringup \
    ros-foxy-nav2-bt-navigator \
    ros-foxy-nav2-map-server \
    ros-foxy-nav2-planner \
    ros-foxy-nav2-controller \
    ros-foxy-nav2-behaviors \
    ros-foxy-nav2-lifecycle-manager \
    ros-foxy-nav2-msgs \
    ros-foxy-navigation2

echo ""
echo "[3/5] Installing SLAM & TF packages..."
sudo apt install -y \
    ros-foxy-slam-toolbox \
    ros-foxy-twist-mux \
    ros-foxy-pointcloud-to-laserscan \
    ros-foxy-robot-state-publisher \
    ros-foxy-joint-state-publisher \
    ros-foxy-tf2-ros \
    ros-foxy-tf2-geometry-msgs

echo ""
echo "[4/5] Installing Perception packages..."
sudo apt install -y \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-message-filters \
    ros-foxy-vision-msgs

echo ""
echo "[5/5] Installing Additional dependencies..."
sudo apt install -y \
    ros-foxy-realsense2-camera \
    ros-foxy-diagnostic-updater \
    python3-colcon-common-extensions \
    python3-rosdep

# Install system dependencies for audio (optional - voice commands)
echo ""
echo "[Optional] Installing audio dependencies for voice commands..."
sudo apt install -y portaudio19-dev || echo "Warning: portaudio not installed (voice commands may not work)"

# Install explore_lite if available
echo ""
echo "Attempting to install explore_lite..."
sudo apt install -y ros-foxy-explore-lite 2>/dev/null || {
    echo ""
    echo "Note: ros-foxy-explore-lite not available via apt."
    echo "You may need to build it from source if exploration is needed:"
    echo "  cd ~/ros2_ws/src"
    echo "  git clone -b foxy https://github.com/robo-friends/m-explore-ros2.git"
    echo "  cd ~/ros2_ws && colcon build --packages-select explore_lite"
}

echo ""
echo "=============================================="
echo "  ROS2 Foxy Dependencies Installation Complete!"
echo "=============================================="
echo ""
echo "Next steps:"
echo "  1. Setup Python virtual environment: ./setup_venv.sh"
echo "  2. Download ML models: ./download_models.sh"
echo "  3. Build workspace: cd ~/ros2_ws && colcon build"
echo ""
