#!/bin/bash
# Consolidated ML dependencies installation script
# Detects GPU, installs appropriate PyTorch, and installs lerobot

set -e

echo "=== ML Dependencies Installation ==="
echo ""

# Step 1: Detect GPU
echo "Step 1: Detecting GPU..."
IS_RTX5090=false

if command -v nvidia-smi &> /dev/null; then
    GPU_MODEL=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n 1)
    echo "Detected GPU: $GPU_MODEL"
    
    if [[ "$GPU_MODEL" == *"5090"* ]] || [[ "$GPU_MODEL" == *"RTX 5090"* ]]; then
        IS_RTX5090=true
        echo "RTX 5090 (Blackwell) detected - will install PyTorch 2.7.0+cu128"
    else
        echo "Standard GPU detected - will install standard PyTorch"
    fi
else
    echo "WARNING: nvidia-smi not found. Assuming CPU-only or no NVIDIA GPU."
    echo "Will install standard PyTorch (CPU-compatible)"
fi

echo ""

# Step 2: Install PyTorch
echo "Step 2: Installing PyTorch..."
if [ "$IS_RTX5090" = true ]; then
    echo "Installing RTX 5090 PyTorch (2.7.0+cu128)..."
    pip install \
        torch==2.7.0+cu128 \
        torchaudio==2.7.0+cu128 \
        torchvision==0.22.0+cu128 \
        --index-url https://download.pytorch.org/whl/cu128
    # Install torchcodec separately from standard PyPI index
    # Version 0.4.0 is the tested version for RTX 5090 with PyTorch 2.7.0+cu128
    # See docs/so_arm_demo.md for the documented dependency versions
    echo "Installing torchcodec from standard PyPI index..."
    pip install torchcodec==0.4.0
else
    echo "Installing standard PyTorch..."
    pip install torch torchvision torchaudio
fi

echo ""

# Step 3: Install lerobot
# Note: Installed via pip due to rerun-sdk wheel compatibility issues with pixi pypi-dependencies
# Version 0.3.3 is pinned to match the lerobot repository version (v0.3.3) used in this project
echo "Step 3: Installing lerobot..."
pip install lerobot==0.3.3

echo ""
echo "=== Installation Complete ==="
echo ""
echo "Installed packages:"
pip list | grep -E "(torch|lerobot)" || true
