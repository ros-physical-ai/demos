# Installation

This guide covers installing the workspace and its dependencies.

## Requirements

- A Linux distribution (any recent x86_64 distro). The Pixi-managed environment bundles ROS 2, Gazebo, and all dependencies, so a specific Ubuntu release is **not** required.
- [Pixi](https://pixi.sh/latest/installation/) (recommended) — manages ROS 2, Gazebo, and all dependencies automatically.
- LibSerial — required by `feetech_ros2_driver` (only needed for real hardware). Pixi installs it for you; for the manual install path below use `sudo apt install -y libserial-dev`.

### GPU

- **Simulation (Gazebo / MuJoCo):** no GPU is strictly required — both simulators run with software rendering — but any GPU with a working OpenGL driver is recommended for smooth rendering.
- **ML inference & training (LeRobot):** training and inference run on [PyTorch](https://pytorch.org/) (LeRobot's deep-learning backend), so a CUDA-capable **NVIDIA GPU** is strongly recommended — training is impractically slow on CPU. The `install-ml-deps` task auto-detects your GPU via `nvidia-smi` and installs a matching PyTorch build: a CUDA wheel when an NVIDIA GPU is present, or a CPU-only build otherwise. CPU is fine for quick tests but not for real workloads.

## Install with Pixi (recommended)

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

See the [Development Guide](development.md) for the full Pixi-based development workflow.

## Alternative: manual install without Pixi

If you prefer a system-wide ROS 2 installation:

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y libserial-dev python3-vcstool
mkdir ~/ws_pai/src -p && cd ~/ws_pai/src
git clone https://github.com/ros-physical-ai/demos
cd demos
vcs import external < pai.repos --recursive
cd ~/ws_pai
rosdep install --from-paths src --ignore-src --rosdistro lyrical -yir
source /opt/ros/lyrical/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

When using this approach, source the workspace before running demos:

```bash
source ~/ws_pai/install/setup.bash
```

## Middleware: rmw_zenoh

> [!NOTE]
> This project uses [rmw_zenoh](https://github.com/ros2/rmw_zenoh) as the default ROS 2 middleware.
> When using Pixi, this is configured automatically. For manual installs, install it via
> `sudo apt install ros-lyrical-rmw-zenoh-cpp` and `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
> Ensure the Zenoh router is running: `ros2 run rmw_zenoh_cpp rmw_zenohd` (or `pixi run zenoh-router`).
