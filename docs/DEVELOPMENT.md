# Development Guide

You can use [Pixi](https://pixi.sh/latest/installation/) to create an isolated workspace that doesn't depend on system-wide ROS installations or external colcon build/install directories.

Install Pixi first by following [its official documentation](https://pixi.prefix.dev/latest/installation/).

## Prerequisites

The following must be installed system-wide. 
See [README.md](../README.md) for installation instructions:

_Dependent repos_: Installed via `vcs import . < pai.repos --recursive`

_libserial-dev_: Required for feetech_ros2_driver. Install via:
  ```bash
  sudo apt update && sudo apt install -y libserial-dev
  ```

NVIDIA drivers and CUDA toolkit are also system components.
Pixi tasks are provided to facilitate their installation (see instructions below).

ROS 2 Kilted dependencies are automatically installed via Pixi when you run `pixi install`. 


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

Build the workspace directly from the repository root folder:

```bash
# Navigate to the repository root (where pixi.toml is located)
cd demos

# Build the workspace using Pixi task
pixi run build
```

### 3. Run Demo

The workspace is configured to use Zenoh middleware automatically. The `RMW_IMPLEMENTATION` environment variable is set to `rmw_zenoh_cpp`, and tasks will automatically start the Zenoh router if it's not already running.

Launch the Gazebo simulation (ROS environment is automatically sourced, Zenoh router started automatically if needed):
```bash
pixi run so-arm-gz
```

Optionally, you can start the Zenoh router manually in a separate terminal if you prefer:
```bash
pixi run start_zenoh
```

### 4. Interactive Mode with Pixi Shell

For interactive development, you can use `pixi shell` to enter an interactive shell with the environment activated:

```bash
cd demos
pixi shell
```

Once in the shell, the ROS environment is automatically sourced and `RMW_IMPLEMENTATION=rmw_zenoh_cpp` is set. You can run commands (e.g., `colcon build`, Python scripts) directly.
This is useful for interactive debugging, testing, and running multiple commands.

Note: When running ROS 2 commands manually in the shell, ensure the Zenoh router is running (use `pixi run start_zenoh` in another terminal, or use `bash scripts/run_with_zenoh.sh <command>`).

For example, to launch the LeRobot inference node:

Terminal 1 (start Zenoh router):
```bash
pixi run start_zenoh
```

Terminal 2 (run inference node):
```bash
pixi shell

# Replace the model path to match your environment
python3 pai_bringup/scripts/lerobot_inference_node \ 
  --ros-args-p policy_path:=outputs/train/act_move_to_cube/checkpoints/last/pretrained_model
```

Alternatively, use the pixi task which automatically ensures Zenoh is running:
```bash
pixi run lerobot-inference
```

For more details on training models, inference parameters, and usage, see [so_arm_demo.md](./so_arm_demo.md).

Additional resources for using Pixi can be found at this [blog](https://jafarabdi.github.io/blog/2025/ros2-pixi-dev/).

## FAQ

### Warning: "Could not find activation scripts: install/setup.bash"

This warning appears on initial setup because `install/setup.bash` is created by `colcon build`. It's harmless and will disappear after running `pixi run build`.

### After updating pixi.toml

After modifying `pixi.toml` to add or update dependencies, run `pixi install` to install the new dependencies before building or running tasks.

### Gazebo Warnings and Errors

Common Gazebo-specific warnings and errors that may appear during simulation:

**"Trying to set debug visualization mode while GI is disabled"**: This error occurs when Gazebo's Global Illumination (GI) system tries to set debug visualization before GI is fully initialized. 

**"libEGL warning: egl: failed to create dri2 screen"**: This warning appears when Gazebo cannot access the GPU directly. 

The simulation will still run using software rendering, but without hardware acceleration.

For resolution steps for these, see:
- [Gazebo Rendering Plugin Documentation](https://gazebosim.org/api/rendering/3/renderingplugin.html)
- [Gazebo Rendering Documentation](https://gazebosim.org/api/rendering/)
- [Gazebo Sim Troubleshooting](https://gazebosim.org/docs/citadel/troubleshooting)
