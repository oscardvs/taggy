#!/bin/bash
# =============================================================================
# Taggy - Complete Installation Script for Go2 Robot
# =============================================================================
# One-script installation for all Taggy dependencies on ROS2 Foxy
# Supports both x86_64 (laptop) and ARM64 (Jetson/Go2) architectures
#
# This script will:
#   1. Install ROS2 Foxy system dependencies
#   2. Create Python virtual environment
#   3. Install Python packages (ARM-aware)
#   4. Download ML models (YOLO11, Vosk)
#   5. Build the ROS2 workspace
#
# Usage:
#   ./install.sh           # Full installation
#   ./install.sh --no-build  # Skip workspace build
#
# Requirements:
#   - Ubuntu 20.04 (x86_64 or ARM64)
#   - ROS2 Foxy installed
#   - JetPack (for Jetson/Go2)
#   - Internet connection
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${SCRIPT_DIR}/.."

# Detect workspace: scripts are in src/taggy/scripts/ (Go2) or src/scripts/ (dev)
# Check if parent dir is named "taggy" to determine structure
PARENT_DIR_NAME=$(basename "${REPO_DIR}")
if [ "${PARENT_DIR_NAME}" = "taggy" ]; then
    # Go2 structure: /home/taggy_ws/src/taggy/scripts/
    WORKSPACE_DIR="${SCRIPT_DIR}/../../.."
    SRC_PACKAGES_PATH="src/taggy"
else
    # Dev structure: /home/user/ros2_ws/src/scripts/
    WORKSPACE_DIR="${SCRIPT_DIR}/../.."
    SRC_PACKAGES_PATH="src"
fi
VENV_PATH="${HOME}/taggy_venv"
NO_BUILD=false

# Detect architecture
ARCH=$(uname -m)
IS_ARM=false
IS_JETSON=false

if [[ "$ARCH" == "aarch64" ]] || [[ "$ARCH" == "arm64" ]]; then
    IS_ARM=true
    if [ -f /etc/nv_tegra_release ] || [ -d /usr/local/cuda ]; then
        IS_JETSON=true
    fi
fi

# Parse arguments
for arg in "$@"; do
    case $arg in
        --no-build)
            NO_BUILD=true
            shift
            ;;
    esac
done

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║              TAGGY - Go2 Robot Installation                  ║"
echo "║                     ROS2 Foxy Edition                        ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "Workspace: ${WORKSPACE_DIR}"
echo "Repo Dir: ${REPO_DIR}"
echo "Packages Path: ${SRC_PACKAGES_PATH}"
echo "Venv Path: ${VENV_PATH}"
echo "Architecture: ${ARCH}"
if [ "$IS_JETSON" = true ]; then
    echo "Platform: NVIDIA Jetson (Go2 Robot)"
elif [ "$IS_ARM" = true ]; then
    echo "Platform: ARM64"
else
    echo "Platform: x86_64 (Laptop/Desktop)"
fi
echo ""

# =============================================================================
# Pre-flight checks
# =============================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Pre-flight Checks"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "OS: ${PRETTY_NAME}"
fi

# Check ROS2 Foxy
if [ -f "/opt/ros/foxy/setup.bash" ]; then
    echo "✓ ROS2 Foxy found"
else
    echo "✗ ROS2 Foxy NOT found!"
    echo ""
    echo "Please install ROS2 Foxy first:"
    echo "  sudo apt update"
    echo "  sudo apt install ros-foxy-desktop"
    exit 1
fi

# Check Python
echo "Python: $(python3 --version)"

# Check JetPack on Jetson
if [ "$IS_JETSON" = true ]; then
    echo ""
    echo "Jetson detected - checking JetPack components..."
    
    # Check CUDA
    if [ -d /usr/local/cuda ]; then
        CUDA_VERSION=$(cat /usr/local/cuda/version.txt 2>/dev/null | head -1 || nvcc --version | grep release | awk '{print $5}' | cut -d',' -f1)
        echo "✓ CUDA: ${CUDA_VERSION}"
    else
        echo "⚠ CUDA not found"
    fi
    
    # Check TensorRT
    if dpkg -l | grep -q tensorrt; then
        TRT_VERSION=$(dpkg -l | grep tensorrt | head -1 | awk '{print $3}')
        echo "✓ TensorRT: ${TRT_VERSION}"
    else
        echo "⚠ TensorRT not found"
    fi
    
    # Check PyTorch
    python3 -c "import torch; print(f'✓ PyTorch: {torch.__version__}')" 2>/dev/null || echo "⚠ PyTorch not found in system"
    
    # Check OpenCV
    python3 -c "import cv2; print(f'✓ OpenCV: {cv2.__version__}')" 2>/dev/null || echo "⚠ OpenCV not found in system"
fi

echo ""

# =============================================================================
# Step 1: ROS2 Dependencies
# =============================================================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  [1/5] Installing ROS2 System Dependencies"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

bash "${SCRIPT_DIR}/install_ros2_deps.sh"

# =============================================================================
# Step 2: Python Virtual Environment
# =============================================================================
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  [2/5] Setting up Python Virtual Environment"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Non-interactive venv setup
sudo apt install -y python3-venv python3-pip python3-dev

if [ ! -d "${VENV_PATH}" ]; then
    python3 -m venv "${VENV_PATH}" --system-site-packages
fi

source "${VENV_PATH}/bin/activate"
pip install --upgrade pip setuptools wheel

# Install packages based on architecture
if [ "$IS_JETSON" = true ]; then
    echo "Installing for Jetson (using system PyTorch/OpenCV)..."
    pip install numpy PyYAML
    pip install ultralytics --no-deps
    pip install vosk || true
    pip install pyaudio || true
    pip install matplotlib pandas seaborn tqdm pillow requests psutil py-cpuinfo
else
    echo "Installing for ${ARCH}..."
    pip install numpy PyYAML
    pip install opencv-python opencv-contrib-python
    pip install ultralytics
    pip install vosk pyaudio || true
fi

echo "✓ Virtual environment ready at ${VENV_PATH}"

# =============================================================================
# Step 3: Download ML Models
# =============================================================================
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  [3/5] Downloading ML Models"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

bash "${SCRIPT_DIR}/download_models.sh"

# =============================================================================
# Step 4: Initialize rosdep
# =============================================================================
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  [4/5] Initializing rosdep"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
    sudo rosdep init || true
fi
rosdep update

# Install any missing rosdep dependencies
cd "${WORKSPACE_DIR}"
rosdep install --from-paths "${SRC_PACKAGES_PATH}" --ignore-src -r -y || true

# =============================================================================
# Step 5: Build Workspace
# =============================================================================
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  [5/5] Building ROS2 Workspace"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if [ "$NO_BUILD" = true ]; then
    echo "Skipping build (--no-build flag set)"
else
    cd "${WORKSPACE_DIR}"
    source /opt/ros/foxy/setup.bash
    colcon build --symlink-install
    echo "✓ Workspace built successfully"
fi

# =============================================================================
# Complete!
# =============================================================================
echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║                                                              ║"
echo "║              INSTALLATION COMPLETE! 🎉                       ║"
echo "║                                                              ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""
echo "To start using Taggy, run these commands:"
echo ""
echo "  # Activate environment"
echo "  source /opt/ros/foxy/setup.bash"
echo "  source ${VENV_PATH}/bin/activate"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
echo ""
echo "  # Or add to ~/.bashrc for automatic activation:"
echo "  echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc"
echo "  echo 'source ${VENV_PATH}/bin/activate' >> ~/.bashrc"
echo "  echo 'source ${WORKSPACE_DIR}/install/setup.bash' >> ~/.bashrc"
echo ""
echo "Repo location: ${REPO_DIR}"
echo "Workspace: ${WORKSPACE_DIR}"
echo ""
echo "  # Launch robot control"
echo "  ros2 launch go2_bringup go2_control.launch.py"
echo ""

if [ "$IS_JETSON" = true ]; then
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  JETSON/GO2 NOTES"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "  • PyTorch/OpenCV use JetPack system libraries"
    echo "  • First perception run will build TensorRT engine (~5-10 min)"
    echo "  • YOLO11 tracker uses JetPack's CUDA for acceleration"
    echo ""
fi

echo "See SETUP.MD for full launch instructions."
echo ""
