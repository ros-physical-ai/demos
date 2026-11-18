# Installation

This guide covers installing the AIC application and the simulator it drives.

## Requirements

- A Linux x86_64 distribution. The Pixi-managed environment bundles ROS 2 Kilted and every host dependency, so any recent distribution works. The simulator image is published for amd64 only.
- [Pixi](https://pixi.sh/latest/installation/).
- [Docker Engine](https://docs.docker.com/engine/install/) with the compose plugin, usable by your user without `sudo`.
- An X11 display for the Gazebo window. Headless operation works without one.

### GPU

- **Simulator rendering.** The three wrist cameras render inside the container. With the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) and `pixi run sim-up-gpu` they publish at 20 Hz, the rate the contract records at. Without a GPU in the container they render in software at well under 1 Hz, which is enough for smoke tests and little else.
- **ML inference and training.** LeRobot runs on PyTorch; a GPU is strongly recommended. The `install-ml-deps` task auto-detects the GPU and installs a matching PyTorch build.

## Install with Pixi

```bash
git clone https://github.com/ros-physical-ai/demos
cd demos/apps/aic
pixi run install-deps
pixi run build
```

`install-deps` runs `install-source-deps`, which imports the shared Rosetta sources and the pinned AIC checkout into `external/` and hides every AIC package except `aic_control_interfaces` from colcon, and `install-ml-deps`, which installs PyTorch and LeRobot for the detected GPU. `build` compiles Rosetta, that package and `pai_aic` in about a minute.

## Pull and start the simulator

```bash
xhost +local:
pixi run sim-up-gpu   # or: pixi run sim-up
pixi run sim-logs     # until both controllers report "Configured and activated"
```

The first start pulls the image, which is several gigabytes. The scene comes from `.env` in this directory; copy [`.env.example`](../.env.example) to change the task board rails or the cable type.

## Verify

```bash
pixi run check-interfaces   # host-built messages match the image
pixi run sim-smoke          # joints and cameras publish, the contract loads
pixi run teleop             # drive the arm; Esc quits
```

## Middleware

Both sides use `rmw_zenoh_cpp`. The container starts the zenoh router on port 7447, the only router in the system; host nodes are its peers and need no configuration beyond what the environment sets: shared memory off, `ROS_DOMAIN_ID=0`, and the session config in [`config/zenoh/session.json5`](../config/zenoh/session.json5).
