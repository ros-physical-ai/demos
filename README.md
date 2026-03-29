# ROS Physical AI Demos

## Requirements

- [Ubuntu 24.04](https://ubuntu.com/download/desktop)
- [Pixi](https://pixi.sh/latest/installation/) (recommended) — manages ROS 2, Gazebo, and all dependencies automatically
- **NVIDIA GPU** (for ML inference):
  - RTX 5090 (Blackwell): Driver version 570+ and CUDA 12.8+ recommended
  - Other GPUs: Compatible driver and CUDA version
- `libserial-dev` — required by feetech_ros2_driver: `sudo apt install -y libserial-dev`

> [!NOTE]
> ROS 2 `Jazzy Jalisco` is also supported but we recommend `Kilted` to benefit from simulation improvements in `Gazebo Ionic` which pairs with `Kilted` together with improvements in `ros2_control`.

## Install

```bash
git clone https://github.com/ros-physical-ai/demos
cd demos
vcs import external < pai.repos --recursive
pixi install
pixi run build
```

To install ML dependencies (PyTorch, LeRobot — automatically detects your GPU):

```bash
pixi run install-ml-deps
```

See [DEVELOPMENT.md](./docs/DEVELOPMENT.md) for the full Pixi-based development workflow.

<details>
<summary><strong>Alternative: manual install without Pixi</strong></summary>

If you prefer a system-wide ROS 2 installation:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y libserial-dev python3-vcstool
mkdir ~/ws_pai/src -p && cd ~/ws_pai/src
git clone https://github.com/ros-physical-ai/demos
cd demos
vcs import external < pai.repos --recursive
cd ~/ws_pai
rosdep install --from-paths src --ignore-src --rosdistro kilted -yir
source /opt/ros/kilted/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

When using this approach, source the workspace before running demos:

```bash
source ~/ws_pai/install/setup.bash
```

</details>

> [!NOTE]
> This project uses [rmw_zenoh](https://github.com/ros2/rmw_zenoh) as the default ROS 2 middleware.
> When using Pixi, this is configured automatically. For manual installs, install it via
> `sudo apt install ros-kilted-rmw-zenoh-cpp` and `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
> Ensure the Zenoh router is running: `ros2 run rmw_zenoh_cpp rmw_zenohd` (or `pixi run start_zenoh`).

## Packages

### This repository

| Package                 | Description                                                                                                                                                 |
| ----------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **pai_bringup**         | Main bringup package — launches the SO-ARM101 in Gazebo, MuJoCo, or on real hardware with ros2_control, RViz, camera bridge, and optional LeRobot inference |
| **pai_leader_teleop**   | Leader-follower teleoperation — brings up a physical leader SO-ARM101 to control a follower arm via ros2_control                                            |
| **pai_data_collection** | Configuration and scripts for collecting demonstration datasets via the Rosetta ROS 2–LeRobot bridge                                                        |
| **pai_description**     | Scene-level SDF world descriptions for the demo environments                                                                                                |
| **pai_assets**          | Shared 3D model assets (meshes, textures) used by the demo scenes                                                                                           |

### External (imported via `pai.repos`)

| Source                                                                                                            | Description                                                                           |
| ----------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| [ros2_so_arm](https://github.com/JafarAbdi/ros2_so_arm)                                                           | URDF descriptions, MoveIt config, Gazebo support, and utilities for the SO-ARM robots |
| [feetech_ros2_driver](https://github.com/legalaspro/feetech_ros2_driver)                                          | ros2_control hardware interface for Feetech servo motors                              |
| [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)                                        | ros2_control integration with the MuJoCo physics simulator                            |
| [rosetta](https://github.com/iblnkn/rosetta) / [rosetta_interfaces](https://github.com/iblnkn/rosetta_interfaces) | ROS 2–LeRobot bridge for recording demonstration datasets                             |
| [lerobot-robot-rosetta](https://github.com/iblnkn/lerobot-robot-rosetta)                                          | LeRobot Robot plugin for Rosetta — bridges ROS 2 topics to LeRobot's Robot interface  |

## Launching the SO-ARM101

### Gazebo

![](./docs/media/so_arm_gz.png)

```bash
ros2 launch pai_bringup so_arm_gz_bringup.launch.py
```

With Pixi: `pixi run so-arm-gz`

### MuJoCo

![](./docs/media/so_arm_mujoco.png)

```bash
ros2 launch pai_bringup so_arm_mujoco_bringup.launch.py
```

With Pixi: `pixi run so-arm-mujoco`

### Real hardware

```bash
ros2 launch pai_bringup so_arm_real_bringup.launch.py
```

With Pixi: `pixi run so-arm-real`

### Leader arm teleoperation

You can use a leader SO-ARM101 to teleoperate the follower arm (sim or real):

```bash
ros2 launch pai_leader_teleop leader_bringup.launch.py
```

With Pixi: `pixi run so-arm-leader`

## Demos

### Pick and Place with SO-ARM

A simple demonstration of training and running a policy with an SO-ARM in sim and real.

For instructions on training a policy and running inference see [this guide](./demos/so_arm_101/rosetta_end_to_end_demo.md).

## Linting & Pre-commit

This repository uses [pre-commit](https://pre-commit.com/) to enforce consistent code quality. The following hooks are configured:

- **General**: trailing whitespace, end-of-file fixer, YAML/XML validation, large file check, merge conflict markers
- **Python**: [Ruff](https://docs.astral.sh/ruff/) for linting and formatting
- **Shell**: [ShellCheck](https://www.shellcheck.net/) for static analysis
- **YAML/Markdown**: [Prettier](https://prettier.io/) for formatting
- **CMake**: [cmake-lint](https://cmake-format.readthedocs.io/) for CMakeLists.txt files

### Setup

Install the git hooks so they run automatically on every commit:

```bash
pre-commit install
```

With Pixi: `pixi run -e default pre-commit install`

### Usage

Hooks will run automatically on staged files when you `git commit`. To run all hooks on all files manually:

```bash
pre-commit run --all-files
```

With Pixi: `pixi run lint`

## External demos

Other demos: fully open-source physical AI projects on ROS.

- [Agentic mobile manipulator](https://github.com/RobotecAI/agentic-mobile-manipulator), a comprehensive demo project using a hardware-in-the-loop setup with O3DE and all the software and inference running on-board.
