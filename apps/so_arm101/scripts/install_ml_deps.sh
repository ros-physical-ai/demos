#!/bin/bash
# Consolidated ML dependencies installation script
# Detects GPU, installs appropriate PyTorch.

set -e

PYTHON=python3
PIP_CMD="$PYTHON -m pip"

echo "=== ML Dependencies Installation ==="
echo ""

# Step 1: Detect hardware backend and Python extension support
echo "Step 1: Detecting hardware backend..."
IS_RTX5090=false
IS_NVIDIA=false
IS_XPU=false

HAVE_XPU_PYTHON_LIBS=false
if command -v "$PYTHON" >/dev/null 2>&1; then
    HAVE_XPU_PYTHON_LIBS=$($PYTHON - <<'PY'
import importlib.util
print(importlib.util.find_spec('intel_extension_for_pytorch') is not None or importlib.util.find_spec('torch_xpu') is not None)
PY
    )
fi

if [ "$HAVE_XPU_PYTHON_LIBS" = "True" ]; then
    echo "Detected XPU Python extension packages in current environment"
else
    echo "No XPU Python extension packages detected in current environment"
fi

# NVIDIA detection takes precedence on hybrid systems with both NVIDIA and Intel GPUs.
if command -v nvidia-smi &> /dev/null; then
    IS_NVIDIA=true
    GPU_MODEL=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -n 1)
    echo "Detected GPU: $GPU_MODEL"

    if [[ "$GPU_MODEL" == *"5090"* ]] || [[ "$GPU_MODEL" == *"RTX 5090"* ]]; then
        IS_RTX5090=true
        echo "RTX 5090 (Blackwell) detected - will install PyTorch 2.9.1+cu130 (sm_120 native; requires open kernel module)"
    else
        echo "Standard NVIDIA GPU detected - will install standard PyTorch"
    fi
else
    echo "No NVIDIA GPU detected"
fi

# Intel XPU detection: only consider XPU if no NVIDIA GPU was detected.
# Explicit opt-in via INTEL_XPU=1 still allows XPU selection on non-NVIDIA systems.
if [ "$IS_NVIDIA" = true ]; then
    echo "NVIDIA GPU detected - prioritizing CUDA installation over Intel XPU"
elif [ "${INTEL_XPU=0}" = "1" ]; then
    IS_XPU=true
    echo "Intel XPU explicitly requested - will install PyTorch XPU build"
elif command -v lspci >/dev/null 2>&1 && lspci -nn | grep -qiE 'Intel Corporation .* (VGA|3D|Graphics|Processing accelerators|NPU)'; then
    IS_XPU=true
    echo "Intel GPU/NPU hardware detected - will install PyTorch XPU build"
elif [ -d /opt/intel/oneapi ]; then
    IS_XPU=true
    echo "Intel OneAPI runtime detected - will install PyTorch XPU build"
fi

echo ""

# Step 2: Install PyTorch
echo "Step 2: Installing PyTorch..."
if [ "$IS_NVIDIA" = true ]; then
    if [ "$IS_RTX5090" = true ]; then
        echo "Installing RTX 5090 PyTorch (2.9.1+cu130)..."
        $PIP_CMD install \
            torch==2.9.1+cu130 \
            torchaudio==2.9.1+cu130 \
            torchvision==0.24.1+cu130 \
            --index-url https://download.pytorch.org/whl/cu130

        # Install torchcodec separately from standard PyPI index
        # Note: torchcodec 0.4.0 was tested with PyTorch 2.7.1+cu128
        # Verify compatibility with 2.9.1 before using in production
        # See docs/so_arm_demo.md for the documented dependency versions
        echo "Installing torchcodec from standard PyPI index..."
        $PIP_CMD install torchcodec==0.4.0
    else
        echo "Installing standard NVIDIA PyTorch build..."
        $PIP_CMD install --upgrade --index-url https://download.pytorch.org/whl/cu124 \
            torch==2.5.1+cu124 \
            torchvision==0.20.1+cu124 \
            torchaudio==2.5.1+cu124
    fi
elif [ "$IS_XPU" = true ]; then
    echo "Installing PyTorch XPU build..."
    $PIP_CMD install --upgrade --index-url https://download.pytorch.org/whl/xpu \
        torch==2.10.0+xpu \
        torchvision==0.25.0+xpu \
        torchaudio==2.11.0+xpu
    $PIP_CMD install intel-extension-for-pytorch
else
    echo "Installing CPU-only PyTorch build..."
    $PIP_CMD install --upgrade --index-url https://download.pytorch.org/whl/cpu \
        torch==2.10.0+cpu \
        torchvision==0.15.2+cpu \
        torchaudio==2.0.2+cpu
fi

echo ""

# Step 3: Verify backend access
echo "Step 3: Verifying installation..."
$PYTHON - <<'PY'
import torch
print(f'PyTorch version : {torch.__version__}')
print(f'CUDA available  : {torch.cuda.is_available()}')
print(f'XPU available   : {hasattr(torch, "xpu") and torch.xpu.is_available()}')
if torch.cuda.is_available():
    print(f'GPU             : {torch.cuda.get_device_name(0)}')
    print(f'CUDA version    : {torch.version.cuda}')
    x = torch.randn(3, 3).cuda()
    print('GPU tensor test : PASSED')
elif hasattr(torch, 'xpu') and torch.xpu.is_available():
    print('XPU device detected')
    x = torch.randn(3, 3, device='xpu')
    print('XPU tensor test : PASSED')
else:
    print('CPU-only install: no CUDA/XPU backend detected')
    x = torch.randn(3, 3)
    print('CPU tensor test : PASSED')
PY

echo ""
echo "=== Installation Complete ==="
echo ""
echo "Installed packages:"
pip list | grep -E "(torch)" || true
