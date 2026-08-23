# Installation

This guide covers installing the workspace and its dependencies.

## Requirements

- A Linux distribution (any recent x86_64/aarch64 distro) or macOS on Apple Silicon (`osx-arm64`). The Pixi-managed environment bundles ROS 2, Gazebo, `libserial`, and all other dependencies, so no OS-level package manager step is required.
- [Pixi](https://pixi.sh/latest/installation/) (recommended) — manages ROS 2, Gazebo, and all dependencies automatically.

### macOS caveats

- Only Apple Silicon (`osx-arm64`) is supported; Intel Macs are not.
- Real-hardware SO-ARM101 control (`so-arm-real`, `so-arm-leader`) isn't supported on macOS: `feetech_ros2_driver` depends on `libserial`, whose `BaudRate` enum only defines rates above 230400 — including the 1,000,000 baud Feetech servos use — on Linux (`#ifdef __linux__`), since macOS's termios API doesn't expose them the same way. This is an upstream `libserial`/`feetech_ros2_driver` limitation.
- MuJoCo simulation (`so-arm-mujoco`) isn't supported on macOS either: `mujoco_ros2_control`'s own driver code trips AppleClang/libc++'s stricter `-Wsign-conversion`/`-Wfloat-conversion` warnings (treated as errors), which GCC doesn't flag the same way. Use Gazebo (`so-arm-gz`) for simulation on macOS instead.
- Camera nodes (`ros-kilted-usb-cam`) wrap Linux's V4L2 API and are unavailable on macOS. Launch real-hardware demos with `use_cameras:=false` on macOS.
- There's no udev on macOS, so the stable `/dev/so101_follower`-style device symlinks from [Hardware Setup](hardware/udev-rules.md) don't apply. Pass the raw device path directly instead, e.g. `usb_port:=/dev/cu.usbmodemXXXX` (find it via `ls /dev/cu.usbmodem*` with the arm plugged in).

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

## Alternative: manual install without Pixi (Linux only)

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

## Middleware: rmw_zenoh

> [!NOTE]
> This project uses [rmw_zenoh](https://github.com/ros2/rmw_zenoh) as the default ROS 2 middleware.
> When using Pixi, this is configured automatically. For manual installs, install it via
> `sudo apt install ros-kilted-rmw-zenoh-cpp` and `export RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
> Ensure the Zenoh router is running: `ros2 run rmw_zenoh_cpp rmw_zenohd` (or `pixi run start_zenoh`).
