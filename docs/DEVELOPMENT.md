# Development Guide

We use [Pixi](https://pixi.sh/latest/installation/) to create an isolated workspace that doesn't depend on system-wide ROS installations or external colcon build/install directories. ROS 2 Kilted dependencies are automatically installed when you run `pixi install`. Install Pixi first by following its official documentation.

## Prerequisites

The following must be installed system-wide. See [README.md](../README.md) for installation instructions:

_Gazebo Ionic_: Installed system-wide

_libserial-dev_: Required for feetech_ros2_driver. Install via:
  ```bash
  sudo apt update && sudo apt install -y libserial-dev
  ```

NVIDIA drivers and CUDA toolkit are also system components. Pixi tasks are provided to facilitate their installation (see instructions below). ROS 2 Kilted dependencies are automatically installed via Pixi when you run `pixi install`. 


## Quick Start

### 1. Setup Development Environment

Detect your GPU and install appropriate dependencies:

```bash
# Step 1: Install base environment (includes ROS 2 Kilted dependencies)
pixi install

# Step 2: Detect GPU and get installation recommendations
pixi run detect-gpu

# Step 3: Install PyTorch (choose ONE based on your GPU)
# For RTX 5090 (Blackwell):
pixi run install-rtx5090-pytorch
# For standard GPU (RTX 30xx, 40xx, etc.):
# pixi run install-standard-pytorch

# Step 4, Install LeRobot
pixi run install-lerobot
```

### 2. Build

Build the workspace directly from the demos folder:

```bash
# Navigate to the demos folder (where pixi.toml is located)
cd demos

# Build the workspace using Pixi task
pixi run build
```

### 3. Run Demo

Start the Zenoh router (recommended middleware, run in a separate terminal):
```bash
pixi run start_zenoh
```

Launch the Gazebo simulation (ROS environment is automatically sourced):
```bash
pixi run so-arm-gz
```

### 4. Interactive Mode with Pixi Shell

For interactive development, you can use `pixi shell` to enter an interactive shell with the environment activated:

```bash
cd demos
pixi shell
```

Once in the shell, the ROS environment is automatically sourced and you can run commands (e.g., colcon build, Python scripts) directly. This is useful for interactive debugging, testing, and running multiple commands. For example, to launch the LeRobot inference node:

```bash
pixi shell

# Replace the model path to match your environment
python3 pai_bringup/scripts/lerobot_inference_node --ros-args \  -p policy_path:=outputs/train/act_move_to_cube/checkpoints/last/pretrained_model
```

For more details on training models, inference parameters, and usage, see [so_arm_demo.md](./so_arm_demo.md).

Additional resources for using Pixi can be found at this [blog](https://jafarabdi.github.io/blog/2025/ros2-pixi-dev/). 