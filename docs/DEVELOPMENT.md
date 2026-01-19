# Development Guide

## Prerequisites

### System-Level Requirements (Global Installation)

These must be installed on your system before using the demo:

- **NVIDIA GPU Driver**: 
  - RTX 5090: Driver version 570 (not 580) required
  - Other GPUs: Compatible driver for your GPU model
- **CUDA Toolkit**: 
  - RTX 5090: CUDA 12.8 required
  - Other GPUs: Compatible CUDA version for your GPU
- **ROS 2 Kilted**: Installed system-wide (see [README.md](../README.md))
- **Gazebo Ionic**: Installed system-wide (see [README.md](../README.md))

> Note: NVIDIA drivers and CUDA toolkit are system-level dependencies and cannot be installed in virtual environments. PyTorch (installed in the Pixi environment) links against these system libraries.

## Quick Start

### 1. Setup Development Environment via Pixi

Even though this package can be built in a colcon workspace on any compatible system, we introduce a workflow that enables developers to work in a completely isolated system environment via [Pixi](https://pixi.sh/latest/installation/). Pixi's strength is its ability to create reproducible, powerful, and flexible workspaces. This ensures consistency with the supported workflows and obviates the need to install any specific ROS, apt, or pip dependencies locally. Install Pixi by following its official documentation before running any instructions below.

Detect your GPU and install appropriate dependencies:

```bash
# Step 1: Detect GPU and get installation recommendations
pixi run detect-gpu

# Step 2: Install base environment
pixi install

# Step 3: Install PyTorch and LeRobot (choose ONE based on your GPU)
# For RTX 5090 (Blackwell):
pixi run install-rtx5090-pytorch
pixi run install-lerobot
# For standard GPU (RTX 30xx, 40xx, etc.):
# pixi run install-standard-pytorch
# pixi run install-lerobot

# Step 4: Install ROS dependencies
pixi run setup-ros

# Step 5: Install SO-ARM100 dependencies (libserial-dev for feetech_ros2_driver)
# This must be run BEFORE entering pixi shell
pixi run setup-so-arm100
```

### 2. Build

Launch an interactive shell environment and build as usual:

```bash
# Navigate to the demos folder (where pixi.toml is located)
cd ~/ws_pai/src/demos

# Make sure SO-ARM100 dependencies are installed (if not done in Step 5)
# This must be run BEFORE entering pixi shell
pixi run setup-so-arm100

# Build the workspace using Pixi task
pixi run build
```

Or set up colcon mixins first (optional):
```bash
pixi run setup-colcon
pixi run build
```

### 3. Run Demo

Launch the Gazebo simulation:
```bash
pixi run so-arm-gz-kilted
```

Or manually:
```bash
source ~/ws_pai/install/setup.bash
ros2 launch pai_bringup so_arm_gz_bringup.launch.py
```

### 4. Run LeRobot Inference

After launching the simulation, run the inference node in a separate terminal:
```bash
pixi run lerobot-inference
```

This runs the LeRobot inference node with default parameters. To customize parameters, modify the task in `pixi.toml` or run the command directly:
```bash
source ~/ws_pai/install/setup.bash
python3 src/demos/pai_bringup/scripts/lerobot_inference_node --ros-args \
    -p policy_path:=<your_policy_path> \
    -p camera_topic:=/camera \
    -p command_topic:=/forward_position_controller/commands \
    -p task:="Move to blue cube" \
    -p device:=cuda
```

For more details on inference parameters and usage, see [so_arm_demo.md](./so_arm_demo.md#inference-in-gazebo).

Additional resources for using Pixi can be found at this [blog](https://jafarabdi.github.io/blog/2025/ros2-pixi-dev/). 