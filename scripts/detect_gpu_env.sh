#!/bin/bash
# GPU Environment Detection Script
# Detects GPU model and CUDA version to suggest appropriate Pixi feature

set -e

echo "=== GPU Environment Detection ==="
echo ""

# Check if nvidia-smi is available
if ! command -v nvidia-smi &> /dev/null; then
    echo "WARNING: nvidia-smi not found. Assuming CPU-only or no NVIDIA GPU."
    echo "Suggested feature: standard-gpu (or CPU mode)"
    exit 0
fi

# Get GPU model
GPU_MODEL=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n 1)
echo "Detected GPU: $GPU_MODEL"

# Get driver version (CUDA version shown in nvidia-smi is max supported, not installed)
DRIVER_VERSION=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n 1)
echo "Driver Version: $DRIVER_VERSION"

# Check for RTX 5090 (Blackwell architecture)
if [[ "$GPU_MODEL" == *"5090"* ]] || [[ "$GPU_MODEL" == *"RTX 5090"* ]]; then
    echo ""
    echo "✓ RTX 5090 (Blackwell) detected!"
    echo ""
    echo "RECOMMENDED: Install base environment, then RTX 5090 PyTorch"
    echo ""
    echo "Installation commands:"
    echo "  pixi install"
    echo "  pixi run install-rtx5090-pytorch"
    echo ""
    echo ""
    echo "Note: RTX 5090 requirements:"
    echo "  - NVIDIA driver 570 (not 580) - SYSTEM-LEVEL, must be installed globally"
    echo "  - CUDA 12.8 toolkit - SYSTEM-LEVEL, must be installed globally"
    echo "  - PyTorch 2.7.0+cu128 - Installed in Pixi environment (links to system CUDA)"
    exit 0
fi

# Note: CUDA toolkit version detection would require checking system installation
# For now, we rely on GPU model detection

echo ""
echo "Standard GPU detected (not RTX 5090)"
echo ""
echo "RECOMMENDED: Install base environment, then standard PyTorch"
echo ""
echo "Installation commands:"
echo "  pixi install"
echo "  pixi run install-standard-pytorch"
echo ""
echo ""
