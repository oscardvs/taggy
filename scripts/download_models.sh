#!/bin/bash
# =============================================================================
# Taggy - Download ML Models (YOLO11 & Vosk)
# =============================================================================
# Downloads required machine learning models for perception and voice commands
#
# Models:
#   - YOLO11n: Lightweight object detection model (v11 Nano)
#   - Vosk: Offline speech recognition model (English)
#
# Usage: ./download_models.sh
# =============================================================================

set -e

echo "=============================================="
echo "  Taggy - ML Model Downloader"
echo "=============================================="
echo ""

# Directories
YOLO_DIR="/opt/yolo"
VOSK_DIR="$HOME/vosk-models"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="${SCRIPT_DIR}/.."

# Detect workspace structure
PARENT_DIR_NAME=$(basename "${REPO_DIR}")
if [ "${PARENT_DIR_NAME}" = "taggy" ]; then
    # Go2 structure: /home/taggy_ws/src/taggy/scripts/
    WORKSPACE_DIR="${SCRIPT_DIR}/../../.."
else
    # Dev structure: /home/user/ros2_ws/src/scripts/
    WORKSPACE_DIR="${SCRIPT_DIR}/../.."
fi

# =============================================================================
# YOLO11 Model
# =============================================================================
echo "[1/2] Setting up YOLO11 model..."

# Create YOLO directory
if [ ! -d "${YOLO_DIR}" ]; then
    echo "Creating ${YOLO_DIR}..."
    sudo mkdir -p "${YOLO_DIR}"
    sudo chown $USER:$USER "${YOLO_DIR}"
fi

# Download YOLO11n using ultralytics (proper method)
YOLO11_PT="${REPO_DIR}/yolo11n.pt"
if [ ! -f "${YOLO11_PT}" ]; then
    echo "Downloading YOLO11n model via ultralytics..."
    python3 -c "
from ultralytics import YOLO
import shutil

# This downloads the model to the ultralytics cache
model = YOLO('yolo11n.pt')

# Find the downloaded model and copy to workspace
import os
from pathlib import Path

# Get the model path from ultralytics
model_path = model.ckpt_path if hasattr(model, 'ckpt_path') else None

if model_path and os.path.exists(model_path):
    shutil.copy(model_path, '${YOLO11_PT}')
    print(f'Model copied to ${YOLO11_PT}')
else:
    # Alternative: export and save
    model.save('${YOLO11_PT}')
    print(f'Model saved to ${YOLO11_PT}')
"
    if [ -f "${YOLO11_PT}" ]; then
        echo "✓ YOLO11n.pt downloaded to ${YOLO11_PT}"
    else
        echo "✗ Failed to download YOLO11n model"
        echo "  Try manually: python3 -c \"from ultralytics import YOLO; YOLO('yolo11n.pt')\""
    fi
else
    echo "✓ YOLO11n.pt already exists at ${YOLO11_PT}"
fi

# Copy to /opt/yolo for perception node
YOLO11_OPT="${YOLO_DIR}/yolo11n.pt"
if [ -f "${YOLO11_PT}" ] && [ ! -f "${YOLO11_OPT}" ]; then
    echo "Copying YOLO11n to ${YOLO_DIR}..."
    cp "${YOLO11_PT}" "${YOLO11_OPT}"
    echo "✓ YOLO11n.pt copied to ${YOLO11_OPT}"
fi

# Export to ONNX for TensorRT (optional, done on first run if needed)
YOLO11_ONNX="${YOLO_DIR}/yolo11n.onnx"
if [ -f "${YOLO11_PT}" ] && [ ! -f "${YOLO11_ONNX}" ]; then
    echo "Exporting YOLO11n to ONNX format..."
    python3 -c "
from ultralytics import YOLO
model = YOLO('${YOLO11_PT}')
model.export(format='onnx', imgsz=640, opset=12)
" 2>/dev/null || echo "Note: ONNX export skipped (will be done on first perception run)"
    
    # Move exported file if it exists
    if [ -f "${REPO_DIR}/yolo11n.onnx" ]; then
        mv "${REPO_DIR}/yolo11n.onnx" "${YOLO11_ONNX}"
        echo "✓ YOLO11n.onnx exported to ${YOLO11_ONNX}"
    fi
fi

# =============================================================================
# Vosk Model (Speech Recognition)
# =============================================================================
echo ""
echo "[2/2] Setting up Vosk speech recognition model..."

# Create Vosk directory
mkdir -p "${VOSK_DIR}"

VOSK_MODEL_NAME="vosk-model-small-en-us-0.15"
VOSK_MODEL_PATH="${VOSK_DIR}/${VOSK_MODEL_NAME}"

if [ ! -d "${VOSK_MODEL_PATH}" ]; then
    echo "Downloading Vosk model (English, small)..."
    cd "${VOSK_DIR}"
    
    wget -q --show-progress \
        "https://alphacephei.com/vosk/models/${VOSK_MODEL_NAME}.zip" \
        -O "${VOSK_MODEL_NAME}.zip"
    
    echo "Extracting model..."
    unzip -q "${VOSK_MODEL_NAME}.zip"
    rm "${VOSK_MODEL_NAME}.zip"
    
    echo "✓ Vosk model extracted to ${VOSK_MODEL_PATH}"
else
    echo "✓ Vosk model already exists at ${VOSK_MODEL_PATH}"
fi

# Create symlink for easier access
VOSK_SYMLINK="/opt/vosk"
if [ ! -L "${VOSK_SYMLINK}" ] && [ ! -d "${VOSK_SYMLINK}" ]; then
    echo "Creating symlink at ${VOSK_SYMLINK}..."
    sudo mkdir -p "$(dirname ${VOSK_SYMLINK})"
    sudo ln -sf "${VOSK_DIR}" "${VOSK_SYMLINK}"
fi

echo ""
echo "=============================================="
echo "  ML Models Download Complete!"
echo "=============================================="
echo ""
echo "Models installed:"
echo "  - YOLO11 (v11 Nano):"
[ -f "${YOLO11_PT}" ] && echo "      ${YOLO11_PT} (PyTorch)"
[ -f "${YOLO11_OPT}" ] && echo "      ${YOLO11_OPT} (PyTorch copy)"
[ -f "${YOLO11_ONNX}" ] && echo "      ${YOLO11_ONNX} (ONNX)"
echo "  - Vosk:"
echo "      ${VOSK_MODEL_PATH}"
echo ""
echo "Note: YOLO11 models are downloaded via ultralytics library"
echo "which ensures you get the correct official weights."
echo ""
