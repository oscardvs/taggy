#!/bin/bash
# =============================================================================
# Taggy - Python Virtual Environment Setup for ROS2 Foxy
# =============================================================================
# Creates a Python venv compatible with ROS2 Foxy (Python 3.8)
# Handles both x86_64 (laptop) and ARM64 (Jetson/Go2) architectures
#
# Usage:
#   ./setup_venv.sh              # Create venv at ~/taggy_venv
#   ./setup_venv.sh /path/to/venv # Create venv at custom location
#
# After running, add to your .bashrc:
#   source ~/taggy_venv/bin/activate
# =============================================================================

set -e

# Default venv location
VENV_PATH="${1:-$HOME/taggy_venv}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${SCRIPT_DIR}/.."
REQUIREMENTS_FILE="${REPO_DIR}/requirements.txt"

# Detect workspace structure
PARENT_DIR_NAME=$(basename "${REPO_DIR}")
if [ "${PARENT_DIR_NAME}" = "taggy" ]; then
    # Go2 structure: /home/taggy_ws/src/taggy/scripts/
    WORKSPACE_DIR="${SCRIPT_DIR}/../../.."
else
    # Dev structure: /home/user/ros2_ws/src/scripts/
    WORKSPACE_DIR="${SCRIPT_DIR}/../.."
fi

# Detect architecture
ARCH=$(uname -m)
IS_ARM=false
IS_JETSON=false

if [[ "$ARCH" == "aarch64" ]] || [[ "$ARCH" == "arm64" ]]; then
    IS_ARM=true
    # Check if running on Jetson
    if [ -f /etc/nv_tegra_release ] || [ -d /usr/local/cuda ]; then
        IS_JETSON=true
    fi
fi

echo "=============================================="
echo "  Taggy - Python Virtual Environment Setup"
echo "=============================================="
echo ""
echo "ROS2 Distribution: Foxy"
echo "Python Version: $(python3 --version)"
echo "Architecture: ${ARCH}"
echo "ARM: ${IS_ARM}"
echo "Jetson: ${IS_JETSON}"
echo "Venv Location: ${VENV_PATH}"
echo ""

# Check Python version (ROS2 Foxy uses Python 3.8)
PYTHON_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)

if [ "$PYTHON_MAJOR" -lt 3 ] || [ "$PYTHON_MINOR" -lt 8 ]; then
    echo "ERROR: Python 3.8+ required for ROS2 Foxy. Found: Python ${PYTHON_VERSION}"
    exit 1
fi

echo "[1/6] Installing Python venv package..."
sudo apt install -y python3-venv python3-pip python3-dev

echo ""
echo "[2/6] Creating virtual environment at ${VENV_PATH}..."
if [ -d "${VENV_PATH}" ]; then
    echo "Warning: Virtual environment already exists at ${VENV_PATH}"
    read -p "Do you want to remove and recreate it? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "${VENV_PATH}"
    else
        echo "Using existing virtual environment."
    fi
fi

if [ ! -d "${VENV_PATH}" ]; then
    # Use --system-site-packages to access system PyTorch/OpenCV on Jetson
    python3 -m venv "${VENV_PATH}" --system-site-packages
fi

echo ""
echo "[3/6] Activating virtual environment..."
source "${VENV_PATH}/bin/activate"

echo ""
echo "[4/6] Upgrading pip..."
pip install --upgrade pip setuptools wheel

echo ""
echo "[5/6] Installing Python dependencies..."

if [ "$IS_JETSON" = true ]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  JETSON DETECTED - Using JetPack libraries"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "Using pre-installed JetPack libraries:"
    echo "  - PyTorch (from JetPack)"
    echo "  - OpenCV (from JetPack)"
    echo "  - CUDA/TensorRT (from JetPack)"
    echo ""
    
    # Verify PyTorch is available
    python3 -c "import torch; print(f'PyTorch: {torch.__version__}')" 2>/dev/null || {
        echo "WARNING: PyTorch not found in system!"
        echo "Please ensure JetPack is properly installed."
    }
    
    # Verify OpenCV is available
    python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}')" 2>/dev/null || {
        echo "WARNING: OpenCV not found in system!"
        echo "Please ensure JetPack is properly installed."
    }
    
    # Install packages that work on ARM (skip opencv, use system torch)
    pip install numpy PyYAML
    pip install ultralytics --no-deps
    pip install vosk pyaudio || echo "Note: pyaudio may fail if portaudio not installed"
    
    # Install ultralytics dependencies (excluding torch/opencv which come from system)
    pip install matplotlib pandas seaborn tqdm pillow requests psutil py-cpuinfo
    
elif [ "$IS_ARM" = true ]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  ARM DETECTED (non-Jetson)"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    
    # Install with pip, some packages may need building
    pip install numpy PyYAML
    pip install opencv-python-headless  # Headless version works better on ARM
    pip install ultralytics
    pip install vosk pyaudio || echo "Note: pyaudio may fail if portaudio not installed"
    
else
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  x86_64 DETECTED (Laptop/Desktop)"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    
    # Standard pip install for x86_64
    if [ -f "${REQUIREMENTS_FILE}" ]; then
        # Install OpenCV separately since it's commented in requirements
        pip install opencv-python opencv-contrib-python
        pip install -r "${REQUIREMENTS_FILE}"
    else
        pip install numpy opencv-python opencv-contrib-python ultralytics vosk pyaudio PyYAML
    fi
fi

echo ""
echo "[6/6] Verifying installation..."
echo ""

python3 << 'EOF'
import sys
print(f"Python: {sys.version}")
print()

# Check numpy
try:
    import numpy as np
    print(f"✓ NumPy: {np.__version__}")
except ImportError:
    print("✗ NumPy: NOT INSTALLED")

# Check OpenCV
try:
    import cv2
    print(f"✓ OpenCV: {cv2.__version__}")
except ImportError:
    print("✗ OpenCV: NOT INSTALLED")

# Check PyTorch
try:
    import torch
    cuda_status = "CUDA available" if torch.cuda.is_available() else "CPU only"
    print(f"✓ PyTorch: {torch.__version__} ({cuda_status})")
except ImportError:
    print("✗ PyTorch: NOT INSTALLED (required for ultralytics)")

# Check ultralytics
try:
    import ultralytics
    print(f"✓ Ultralytics: {ultralytics.__version__}")
except ImportError:
    print("✗ Ultralytics: NOT INSTALLED")

# Check Vosk
try:
    import vosk
    print(f"✓ Vosk: installed")
except ImportError:
    print("✗ Vosk: NOT INSTALLED")

print()
EOF

echo ""
echo "=============================================="
echo "  Virtual Environment Setup Complete!"
echo "=============================================="
echo ""
echo "To activate the virtual environment, run:"
echo "  source ${VENV_PATH}/bin/activate"
echo ""
echo "Add this to your ~/.bashrc for automatic activation:"
echo "  echo 'source ${VENV_PATH}/bin/activate' >> ~/.bashrc"
echo ""
echo "Combined ROS2 + venv activation:"
echo "  source /opt/ros/foxy/setup.bash"
echo "  source ${VENV_PATH}/bin/activate"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
echo ""

if [ "$IS_JETSON" = true ]; then
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  JETSON NOTES"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "  PyTorch/OpenCV use JetPack system libraries."
    echo "  Do NOT pip install torch or opencv-python!"
    echo ""
    echo "  To verify CUDA:"
    echo "    python3 -c \"import torch; print(torch.cuda.is_available())\""
    echo ""
fi
