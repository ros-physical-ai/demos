# AIC cable insertion

The Record → Train → Deploy pipeline of this repository running against the [AI for Industry Challenge](https://github.com/intrinsic-dev/aic) (AIC) scenario: a UR5e under Cartesian impedance control inserts a fiber cable into a task board in Gazebo. The simulator, controllers and sensor adapter come from the container image AIC publishes. Recording, porting, training and deployment run on the host with the same [Rosetta](https://github.com/iblnkn/rosetta) and [LeRobot](https://github.com/huggingface/lerobot) tools the SO-ARM101 application uses.

This application lives at `apps/aic/`; see the [Applications catalog](../../README.md#applications) for how it fits alongside other applications in this repository.

## Quick start

Requires Linux x86_64, [Pixi](https://pixi.sh/latest/installation/), Docker Engine with the compose plugin, and an X11 display for the Gazebo window. The [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) is strongly recommended: with it the cameras render at the contract's 20 Hz, without it they render in software at well under 1 Hz. See the [Installation Guide](docs/installation.md).

```bash
git clone https://github.com/ros-physical-ai/demos
cd demos/apps/aic
pixi run install-deps
pixi run build
```

Bring the simulator up, then check that the host sees it:

```bash
xhost +local:            # once per session, lets the container open the Gazebo window
pixi run sim-up-gpu      # or `pixi run sim-up` without the NVIDIA toolkit
pixi run check-interfaces
pixi run sim-smoke
```

The Gazebo window shows the UR5e, the task board and a cable held in the gripper. Drive the arm from the keyboard with `pixi run teleop` in its own terminal, and stop the simulator with `pixi run sim-down`.

## How it is put together

```
┌─ simulator container (host networking, ROS 2 Kilted) ─────────────────┐
│  gz sim │ aic_controller │ aic_adapter │ ros_gz_bridge                 │
│  zenoh router on :7447                                                │
└────────────────────────────┬──────────────────────────────────────────┘
                             │  /joint_states, /{left,center,right}_camera/image
                             │  /aic_controller/pose_commands, /clock
┌────────────────────────────┴─ host: this Pixi environment (Kilted) ────┐
│  teleop   episode recorder   rosetta_port   lerobot-train   policy    │
│                                                             runner    │
└───────────────────────────────────────────────────────────────────────┘
```

- The container runs only what AIC publishes, pinned by image digest in [`compose.yaml`](compose.yaml). The scene (task board pose, rail mounts, cable type) comes from [`.env.example`](.env.example): copy it to `.env` to change it.
- The host builds Rosetta, `pai_aic` and one AIC message package, `aic_control_interfaces`. [`scripts/prune_aic_source.sh`](scripts/prune_aic_source.sh) hides the rest of the AIC checkout from colcon.
- `pixi run check-interfaces` proves the host-built message definitions match the image. If the two drift, type hashes differ and the action topic silently never connects.
- The host environment is ROS 2 Kilted on Python 3.12, matching the image, so both sides run the same zenoh release. `ROS_DOMAIN_ID` is pinned to 0 inside the environment for the same reason.

## Tasks

All tasks run from `apps/aic`.

| Task                    | Does                                                                                      |
| ----------------------- | ----------------------------------------------------------------------------------------- |
| `install-deps`          | `install-source-deps` (vcs import and prune) plus `install-ml-deps` (PyTorch and LeRobot) |
| `build`                 | Thin colcon build: Rosetta, `aic_control_interfaces`, `pai_aic`                           |
| `check-interfaces`      | Interface drift guard against the pinned image                                            |
| `sim-up` / `sim-up-gpu` | Start the simulator from the pinned image, without or with the NVIDIA overlay             |
| `sim-down` / `sim-logs` | Stop the simulator / follow its logs                                                      |
| `sim-smoke`             | Host-side check that joints, cameras and the contract are all in order                    |
| `reset-scene`           | Home the arm and put a fresh cable in the gripper, between episodes                       |
| `teleop`                | Keyboard teleop in the tool frame; reads its own terminal only                            |
| `record`                | Rosetta episode recorder with the AIC contract                                            |
| `record-keys`           | Rosetta's keyboard node: `r` starts an episode, `s` stops it                              |
| `port-bags`             | Convert recorded bags to a LeRobot dataset under `datasets/`                              |
| `train`                 | Train an ACT policy on that dataset into `outputs/train/aic_act`                          |
| `deploy`                | Run the trained policy against the simulator through Rosetta's policy runner              |
| `lint` / `clean`        | Pre-commit on the whole repository / remove `build`, `install`, `log`                     |

## Packages

### This application

| Package     | Description                                                                                                                  |
| ----------- | ---------------------------------------------------------------------------------------------------------------------------- |
| **pai_aic** | The Rosetta contract, the twist action codecs, the keyboard teleop, and the record and deploy launch files for the AIC scene |

### External (imported via [`aic.repos`](aic.repos) and the shared [`common/common.repos`](../../common/common.repos), via `pixi run install-source-deps`)

| Source                                                                      | Used for                                                                 |
| --------------------------------------------------------------------------- | ------------------------------------------------------------------------ |
| `aic`                                                                       | `aic_control_interfaces` only; every other package is hidden from colcon |
| `rosetta`, `rosetta_interfaces`, `lerobot_rosetta`, `lerobot_robot_rosetta` | The recording, porting and deployment pipeline                           |

## Guides

| Guide                                                    | What it covers                                            |
| -------------------------------------------------------- | --------------------------------------------------------- |
| [Installation](docs/installation.md)                     | Requirements, Docker and GPU setup, verifying the install |
| [End-to-end pipeline](docs/demos/end-to-end-pipeline.md) | Contract, recording, porting, training and deployment     |

## Troubleshooting

- **No topics on the host, or `sim-smoke` times out.** Run `pixi run check-interfaces`; a mismatch means the image and `aic.repos` are out of step. Then check `pixi run sim-logs` for the controller activation lines.
- **The Gazebo window does not appear.** Run `xhost +local:` on the host before `sim-up`. On Wayland sessions, start under XWayland or run headless with `AIC_GAZEBO_GUI=false`.
- **Cameras publish at under 1 Hz.** The simulator is rendering in software. Install the NVIDIA Container Toolkit and use `pixi run sim-up-gpu`.
- **The teleop does nothing.** It reads keys from its own terminal, so that terminal must have focus. Esc quits and publishes a zero twist.
- **zenoh errors mentioning the session config.** The environment ships its own session config in [`config/zenoh/session.json5`](config/zenoh/session.json5); see the header of that file.
