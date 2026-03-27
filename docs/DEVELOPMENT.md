# Development Guide

You can use [Pixi](https://pixi.sh/latest/installation/) to create an isolated workspace that doesn't depend on system-wide ROS installations or external colcon build/install directories.

Install Pixi first by following [its official documentation](https://pixi.prefix.dev/latest/installation/).

## Prerequisites

The following must be installed system-wide. 
See [README.md](../README.md) for installation instructions:

_Dependent repos_: Installed via `vcs import external < pai.repos --recursive`

_libserial-dev_: Required for feetech_ros2_driver. Install via:
  ```bash
  sudo apt update && sudo apt install -y libserial-dev
  ```

_NVIDIA drivers and CUDA toolkit_: Required for GPU acceleration (system components)

ROS 2 Kilted dependencies are automatically installed via Pixi when you run `pixi install`.


## Quick Start

### 1. Setup Development Environment

Install base environment and ML dependencies:

```bash
# Step 1: Install base environment (includes ROS 2 Kilted dependencies)
pixi install

# Step 2: Install ML dependencies (automatically detects GPU and installs appropriate PyTorch)
pixi run install-ml-deps
```

The `install-ml-deps` task automatically:
- Detects your GPU (RTX 5090 or standard)
- Installs the appropriate PyTorch version

### 2. Build

Build the workspace directly from the repository root folder:

```bash
# Navigate to the repository root (where pixi.toml is located)
cd demos

# Build the workspace using Pixi task
pixi run build
```

### 3. Per-Robot Calibration

Each physical robot has its own calibration. Calibration files are stored under `pai_bringup/config/lerobots/<robot_id>/` with standardized filenames (`follower_arm.json`, `leader_arm.json`).

| Robot ID | Description | Tracked in git? |
|----------|-------------|-----------------|
| `nominal` | Default/CI limits (no physical robot required) | Yes |
| `arm-001`, `arm-002`, ... | Per-unit calibration from LeRobot | No (gitignored) |

**First-time setup for a new robot:**

```bash
# 1. Run LeRobot calibration (writes to ~/.cache/huggingface/lerobot/calibration/)
#    See docs/calibration_guide.md for details.

# 2. Copy calibration JSON into the per-robot directory
mkdir -p pai_bringup/config/lerobots/<robot_id>
cp ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/follower_arm.json \
   pai_bringup/config/lerobots/<robot_id>/follower_arm.json
cp ~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/leader_arm.json \
   pai_bringup/config/lerobots/<robot_id>/leader_arm.json

# 3. Generate follower.yaml and leader.yaml for the feetech_ros2_driver
pixi run python3 pai_bringup/scripts/generate_limits.py \
    --robot-id <robot_id> --arm follower \
    --generate-yaml pai_bringup/config/lerobots/<robot_id>/follower.yaml

pixi run python3 pai_bringup/scripts/generate_limits.py \
    --robot-id <robot_id> --arm leader \
    --generate-yaml pai_bringup/config/lerobots/<robot_id>/leader.yaml

# 4. Rebuild so the nominal config is installed
pixi run build
```

**Launching with a specific robot:**

```bash
# Real hardware — auto-resolves follower.yaml from robot_id
ros2 launch pai_bringup so_arm_real_bringup.launch.py robot_id:=arm-001

# MuJoCo (uses robot's calibration-derived joint limits)
ros2 launch pai_bringup so_arm_mujoco_bringup.launch.py robot_id:=arm-001

# Override with a specific joint_config_file (bypasses robot_id resolution)
ros2 launch pai_bringup so_arm_real_bringup.launch.py \
    joint_config_file:=/absolute/path/to/follower.yaml
```

Omitting `robot_id` defaults to `nominal`. See [calibration_guide.md](./calibration_guide.md) for the full calibration procedure and `generate_limits.py` options.

### 4. Run Demo

The workspace is configured to use Zenoh middleware automatically.
The `RMW_IMPLEMENTATION` environment variable is set to `rmw_zenoh_cpp` via the Pixi activation environment.

Before running any ROS 2 commands, start the Zenoh router in a separate terminal.

Terminal 1 (start Zenoh router):
```bash
pixi run start_zenoh
```

Terminal 2 (launch Gazebo simulation):
```bash
pixi run so-arm-gz
```

MuJoCo simulation (SO-ARM in MuJoCo):
```bash
pixi run so-arm-mujoco
```

### 5. Interactive Mode with Pixi Shell

For interactive development, you can use `pixi shell` to enter an interactive shell with the environment activated.

```bash
cd demos
pixi shell
```

Once in the shell, the ROS environment is automatically sourced and `RMW_IMPLEMENTATION=rmw_zenoh_cpp` is set.
You can run commands (e.g., `colcon build`, Python scripts) directly.
This is useful for interactive debugging, testing, and running multiple commands.

Note: When running ROS 2 commands manually in the shell, ensure the Zenoh router is running.
Start it in a separate terminal using `pixi run start_zenoh`.

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

For more details on training models, inference parameters, and usage, see [so_arm_demo.md](./so_arm_demo.md).

Additional resources for using Pixi can be found at this [blog](https://jafarabdi.github.io/blog/2025/ros2-pixi-dev/).

### Scene Configuration

MuJoCo loads poses from `pai_bringup/config/world/poses.yaml` at launch time. Edit `poses.yaml` to change table, tray, or cube positions, then rebuild and relaunch (or relaunch only with symlink-install). See [config/world/README.md](../pai_bringup/config/world/README.md) for details.

## FAQ

### Warning: "Could not find activation scripts: install/setup.bash"

This warning appears on initial setup because `install/setup.bash` is created by `colcon build`.
It's harmless and will disappear after running `pixi run build`.

### After updating pixi.toml

After modifying `pixi.toml` to add or update dependencies, run `pixi install` to install the new dependencies before building or running tasks.

### Gazebo Warnings and Errors

Common Gazebo-specific warnings and errors that may appear during simulation:

**"Trying to set debug visualization mode while GI is disabled"**: This error occurs when Gazebo's Global Illumination (GI) system tries to set debug visualization before GI is fully initialized. 

**"libEGL warning: egl: failed to create dri2 screen"**: This warning appears when Gazebo cannot access the GPU directly. 

The simulation will still run using software rendering, but without hardware acceleration.

For resolution steps for these, see:
- [Gazebo Rendering Plugin Documentation](https://gazebosim.org/api/rendering/9/renderingplugin.html)
- [Gazebo Sim Troubleshooting](https://gazebosim.org/docs/ionic/troubleshooting/)
