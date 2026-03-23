#!/bin/bash
# Consolidated ML dependencies installation script
# Detects GPU, installs appropriate PyTorch.

set -e

echo "=== ML Dependencies Installation ==="
echo ""

# Step 1: Detect GPU
echo "Step 1: Detecting GPU..."
IS_RTX5090=false
IS_NVIDIA=false

if command -v nvidia-smi &> /dev/null; then
    IS_NVIDIA=true
    GPU_MODEL=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n 1)
    echo "Detected GPU: $GPU_MODEL"

    if [[ "$GPU_MODEL" == *"5090"* ]] || [[ "$GPU_MODEL" == *"RTX 5090"* ]]; then
        IS_RTX5090=true
        echo "RTX 5090 (Blackwell) detected - will install PyTorch 2.9.1+cu130 (sm_120 native; requires open kernel module)"
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
    # RTX 5090 (Blackwell/sm_120) requirements:
    #   - Requires NVIDIA open kernel module (nvidia-dkms-580-open or later)
    #   - Requires CUDA 13.0 driver (580.126.20+)
    #   - cu130 wheels available from PyTorch 2.9.0+ only (cu128 is backward compatible but not native)
    #   - Avoid cu126 wheels: no sm_120 support, falls back to slow compatibility mode
    echo "Installing RTX 5090 PyTorch (2.9.1+cu130)..."
    pip install \
        torch==2.9.1+cu130 \
        torchaudio==2.9.1+cu130 \
        torchvision==0.24.1+cu130 \
        --index-url https://download.pytorch.org/whl/cu130

    # Install torchcodec separately from standard PyPI index
    # Note: torchcodec 0.4.0 was tested with PyTorch 2.7.1+cu128
    # Verify compatibility with 2.9.1 before using in production
    # See docs/so_arm_demo.md for the documented dependency versions
    echo "Installing torchcodec from standard PyPI index..."
    pip install torchcodec==0.4.0
else
    echo "Installing standard PyTorch..."
    pip install torch torchvision torchaudio
fi

echo ""

# Step 3: Verify GPU access (skipped if no NVIDIA GPU detected)
echo "Step 3: Verifying installation..."
if [ "$IS_NVIDIA" = true ]; then
    python3 -c "
import torch
print(f'PyTorch version : {torch.__version__}')
print(f'CUDA available  : {torch.cuda.is_available()}')
if torch.cuda.is_available():
    print(f'GPU             : {torch.cuda.get_device_name(0)}')
    print(f'CUDA version    : {torch.version.cuda}')
    x = torch.randn(3, 3).cuda()
    print(f'GPU tensor test : PASSED')
else:
    print('WARNING: CUDA not available - check driver and PyTorch installation')
"
else
    python3 -c "
import torch
print(f'PyTorch version : {torch.__version__}')
print(f'CPU-only install: CUDA not expected')
x = torch.randn(3, 3)
print(f'CPU tensor test : PASSED')
"
fi

echo ""
echo "=== Installation Complete ==="
echo ""
echo "Installed packages:"
pip list | grep -E "(torch)" || true
