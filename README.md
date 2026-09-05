# ROS Physical AI Demos

Open-source physical AI demos for the [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100) robot arm on ROS 2 — simulation (Gazebo, MuJoCo) and real hardware, with a full Record → Train → Deploy learning pipeline powered by [LeRobot](https://github.com/huggingface/lerobot).

## Quick start

Requires Linux and [Pixi](https://pixi.sh/latest/installation/) (which bundles ROS 2, Gazebo, and all dependencies). An NVIDIA GPU is only needed for ML inference/training, not for simulation. For full requirements and the manual install path, see the [Installation Guide](docs/installation.md).

```bash
git clone https://github.com/ros-physical-ai/demos
cd demos
vcs import external < pai.repos --recursive
pixi install
pixi run install-ml-deps   # PyTorch + LeRobot (auto-detects GPU)
pixi run build
```

Launch the SO-ARM101 in Gazebo (start the Zenoh router first, in its own terminal):

```bash
pixi run zenoh-router  # terminal 1
pixi run so-arm-gz     # terminal 2
```

See [Running the Robot](docs/running-the-robot.md) for MuJoCo and real-hardware bringup.

## Documentation

Full documentation lives in [`docs/`](docs/README.md). Highlights:

| Guide                                                             | What it covers                                                                                                       |
| ----------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| [Installation](docs/installation.md)                              | Requirements, Pixi install, manual install, `rmw_zenoh`                                                              |
| [Development Guide](docs/development.md)                          | Pixi workflow, building, FAQ, troubleshooting                                                                        |
| [Running the Robot](docs/running-the-robot.md)                    | Launching in Gazebo, MuJoCo, or on real hardware                                                                     |
| [Hardware Setup](docs/README.md#hardware-setup)                   | Calibration, udev rules, cameras                                                                                     |
| [Teleoperation](docs/teleoperation.md)                            | Leader arm, RViz interactive marker, phone (WebXR)                                                                   |
| [MCP Interface](docs/mcp.md)                                      | Use an AI agent to control, introspect, or debug the robot via [ROS-MCP](https://github.com/robotmcp/ros-mcp-server) |
| [End-to-End Learning Pipeline](docs/demos/end-to-end-pipeline.md) | Record → Train → Deploy with Rosetta and LeRobot                                                                     |
| [Try a Pre-trained Policy](docs/demos/pretrained-demo.md)         | Skip the slow loop — run a pre-trained ACT policy in Gazebo in minutes                                               |
| [Contributing](docs/contributing.md)                              | Linting and pre-commit hooks                                                                                         |

## Packages

### This repository

| Package                 | Description                                                                                                                                                    |
| ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **pai_bringup**         | Main bringup package — launches the SO-ARM101 in Gazebo, MuJoCo, or on real hardware with ros2_control, RViz, camera bridge, and optional LeRobot inference    |
| **pai_leader_teleop**   | Leader-follower teleoperation — brings up a physical leader SO-ARM101 to control a follower arm via ros2_control                                               |
| **pai_rviz_teleop**     | Interactive-marker differential IK teleoperation in RViz                                                                                                       |
| **pai_phone_teleop**    | Phone-based 6-DoF pose teleoperation over WebXR                                                                                                                |
| **pai_data_collection** | Configuration and scripts for collecting demonstration datasets via the Rosetta ROS 2–LeRobot bridge                                                           |
| **pai_description**     | Scene-level SDF world definitions — single source of truth for both Gazebo (loaded natively) and MuJoCo (converted to MJCF at launch time via `sdformat_mjcf`) |
| **pai_assets**          | Shared 3D model assets (meshes, textures) used by the demo scenes                                                                                              |

### External (imported via `pai.repos`)

| Source                                                                                                            | Description                                                                           |
| ----------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| [ros2_so_arm](https://github.com/ros-physical-ai/ros2_so_arm)                                                     | URDF descriptions, MoveIt config, Gazebo support, and utilities for the SO-ARM robots |
| [feetech_ros2_driver](https://github.com/ros-physical-ai/feetech_ros2_driver)                                     | ros2_control hardware interface for Feetech servo motors                              |
| [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)                                        | ros2_control integration with the MuJoCo physics simulator                            |
| [rosetta](https://github.com/iblnkn/rosetta) / [rosetta_interfaces](https://github.com/iblnkn/rosetta_interfaces) | ROS 2–LeRobot bridge for recording demonstration datasets                             |
| [lerobot-robot-rosetta](https://github.com/iblnkn/lerobot-robot-rosetta)                                          | LeRobot Robot plugin for Rosetta — bridges ROS 2 topics to LeRobot's Robot interface  |

We would like to acknowledge the great work of [JafarAbdi](https://github.com/JafarAbdi) in creating ROS 2 drivers for the SO-ARM robots, and transferring his repositories to the `ros-physical-ai` organization.

## Demos

### Try a Pre-trained Policy in Simulation

New to the repo? Skip the slow Record → Train → Deploy loop and see a working policy in Gazebo in minutes. We provide **60 pre-recorded rosbags**, a **converted LeRobot dataset**, and a **trained ACT policy** — all hosted on the HuggingFace Hub. Just point `rosetta_client_launch.py` at the checkpoint and run inference. See [Try a Pre-trained Policy](docs/demos/pretrained-demo.md) for the full walkthrough.

### End-to-End Learning Pipeline with SO-ARM

Record demonstrations, train a policy, and deploy it on the robot — in simulation or on real hardware, using any input method (leader arm teleoperation, scripted commands, or custom controllers). For the full guide, see [End-to-End Learning Pipeline with Rosetta](docs/demos/end-to-end-pipeline.md).

#### Recording episodes

<table>
<tr>
<td align="center"><b>Simulation</b></td>
<td align="center"><b>Real Hardware</b></td>
</tr>
<tr>
<td>

https://github.com/user-attachments/assets/9bd16f15-358f-44e2-80f8-df01aaca47c0

</td>
<td>

https://github.com/user-attachments/assets/bcc907b0-0914-43be-89ee-5bd161139264

</td>
</tr>
<tr>
<td align="center"><em>Recording episodes in Gazebo via leader arm teleoperation</em></td>
<td align="center"><em>Recording episodes on real SO-ARM101</em></td>
</tr>
</table>

#### Trained policy inference

<table>
<tr>
<td align="center"><b>Simulation</b></td>
<td align="center"><b>Real Hardware</b></td>
</tr>
<tr>
<td>

https://github.com/user-attachments/assets/9183df05-4db4-46b4-90ef-56cfd50b56c2

</td>
<td>

https://github.com/user-attachments/assets/51beafa7-4d85-4a53-b0db-ec593f663850

</td>
</tr>
<tr>
<td align="center"><em>Trained policy running in Gazebo</em></td>
<td align="center"><em>Trained policy running on real SO-ARM101</em></td>
</tr>
</table>

> [!NOTE]
> These videos show an **ACT** policy trained on the recorded episodes. The goal here is to demonstrate the full **Record → Train → Deploy** pipeline — not to showcase optimal policy performance, which depends on the number of episodes, model selection, and hyperparameter tuning.

## External demos

Other fully open-source physical AI projects on ROS:

- [Agentic mobile manipulator](https://github.com/RobotecAI/agentic-mobile-manipulator), a comprehensive demo project using a hardware-in-the-loop setup with O3DE and all the software and inference running on-board.
