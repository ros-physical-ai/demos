# ROS Physical AI Demos

A growing **collection of open-source physical AI demos on ROS 2**, each pairing a
robot and simulator with a full Record → Train → Deploy learning pipeline powered
by [LeRobot](https://github.com/huggingface/lerobot). Every demo shares the same
tooling and the same [Pixi](https://pixi.sh/latest/installation/) workspace, but
is otherwise self-contained.

## Demos

Pick a demo below and follow its guide. Each demo lives in its **own Pixi
environment** with its own setup and build commands, so you only ever build the
one you want — running the SO-ARM101 demo never compiles the AIC demo's
dependencies, and vice versa.

| Demo | Robot | Simulator | Set up & build | Guide |
| ---- | ----- | --------- | -------------- | ----- |
| **SO-ARM101 pick & place** | [SO-ARM101](https://github.com/TheRobotStudio/SO-ARM100) | Gazebo · MuJoCo · real hardware | `pixi run setup` → `pixi run build` | Featured below · [End-to-End Pipeline](docs/demos/end-to-end-pipeline.md) |
| **AIC cable-insertion** | UR5 | Gazebo (built from source) | `pixi run setup-aic` → `pixi run aic-build` | [AIC Scenario](docs/demos/aic-scenario.md) |

> [!NOTE]
> New demos are added as additional Pixi environments + a row in this table.
> The default environment is the **SO-ARM101** demo (conda-provided Gazebo), so a
> plain `pixi install` / `pixi run build` never compiles the heavier from-source
> stacks used by other demos.

## Featured: SO-ARM101 in simulation

Requires Linux and [Pixi](https://pixi.sh/latest/installation/) (which bundles ROS 2, Gazebo, and all dependencies). An NVIDIA GPU is only needed for ML inference/training, not for simulation. For full requirements and the manual install path, see the [Installation Guide](docs/installation.md).

```bash
git clone https://github.com/ros-physical-ai/demos
cd demos
pixi install
pixi run setup             # import the SO-ARM101 demo sources (pai.repos)
pixi run install-ml-deps   # PyTorch + LeRobot (auto-detects GPU)
pixi run build             # build the base demo (conda-provided Gazebo)
```

Launch the SO-ARM101 in Gazebo (start the Zenoh router first, in its own terminal):

```bash
pixi run zenoh-router  # terminal 1
pixi run so-arm-gz     # terminal 2
```

See [Running the Robot](docs/running-the-robot.md) for MuJoCo and real-hardware bringup.
For the **AIC cable-insertion** demo, follow the [AIC Scenario guide](docs/demos/aic-scenario.md) instead.

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
| [AIC Cable-Insertion Scenario](docs/demos/aic-scenario.md)        | The AIC demo end to end — UR5 + from-source Gazebo, setup, record, train, deploy                                     |
| [Contributing](docs/contributing.md)                              | Linting and pre-commit hooks                                                                                         |

## Packages

Packages are grouped by the demo that owns them. Base packages are shared
infrastructure used by the SO-ARM101 demo (and the common pipeline); the AIC
demo adds `pai_aic` on top.

### This repository — base (SO-ARM101 + shared)

| Package                 | Description                                                                                                                                                    |
| ----------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **pai_bringup**         | Main bringup package — launches the SO-ARM101 in Gazebo, MuJoCo, or on real hardware with ros2_control, RViz, camera bridge, and optional LeRobot inference    |
| **pai_leader_teleop**   | Leader-follower teleoperation — brings up a physical leader SO-ARM101 to control a follower arm via ros2_control                                               |
| **pai_rviz_teleop**     | Interactive-marker differential IK teleoperation in RViz                                                                                                       |
| **pai_phone_teleop**    | Phone-based 6-DoF pose teleoperation over WebXR                                                                                                                |
| **pai_data_collection** | Configuration and scripts for collecting demonstration datasets via the Rosetta ROS 2–LeRobot bridge                                                           |
| **pai_description**     | Scene-level SDF world definitions — single source of truth for both Gazebo (loaded natively) and MuJoCo (converted to MJCF at launch time via `sdformat_mjcf`) |
| **pai_assets**          | Shared 3D model assets (meshes, textures) used by the demo scenes                                                                                              |

### This repository — AIC cable-insertion

| Package     | Description                                                                                                                                                                                       |
| ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **pai_aic** | Bridge to the AI for Industry Challenge cable-insertion scenario — reuses demos' Record → Train → Deploy pipeline with a custom Rosetta contract and a `LerobotPolicy` class for AIC-native deploy |

### External (imported via `vcs`)

Sources are imported into `external/` (base) and `external_aic/` (AIC) from two
manifests: `pai.repos` (base, pulled by `pixi run setup`) and `aic.repos` (AIC,
pulled by `pixi run setup-aic`).

| Source                                                                                                            | Demo | Description                                                                           |
| ----------------------------------------------------------------------------------------------------------------- | ---- | ------------------------------------------------------------------------------------- |
| [ros2_so_arm](https://github.com/ros-physical-ai/ros2_so_arm)                                                     | base | URDF descriptions, MoveIt config, Gazebo support, and utilities for the SO-ARM robots |
| [feetech_ros2_driver](https://github.com/ros-physical-ai/feetech_ros2_driver)                                     | base | ros2_control hardware interface for Feetech servo motors                              |
| [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)                                        | base | ros2_control integration with the MuJoCo physics simulator                            |
| [rosetta](https://github.com/iblnkn/rosetta) / [rosetta_interfaces](https://github.com/iblnkn/rosetta_interfaces) | base | ROS 2–LeRobot bridge for recording demonstration datasets                             |
| [lerobot-robot-rosetta](https://github.com/iblnkn/lerobot-robot-rosetta)                                          | base | LeRobot Robot plugin for Rosetta — bridges ROS 2 topics to LeRobot's Robot interface  |
| [aic](https://github.com/intrinsic-dev/aic) + UR5 / from-source Gazebo / ros2_control                             | AIC  | AIC packages (`aic_bringup`, `aic_engine`, `aic_model`, …) and their build deps — see the [AIC Scenario guide](docs/demos/aic-scenario.md) |

We would like to acknowledge the great work of [JafarAbdi](https://github.com/JafarAbdi) in creating ROS 2 drivers for the SO-ARM robots, and transferring his repositories to the `ros-physical-ai` organization.

## SO-ARM101 walkthroughs

The walkthroughs below illustrate the shared **Record → Train → Deploy** pipeline
on the SO-ARM101 demo. For the AIC cable-insertion demo, see its dedicated
[AIC Scenario guide](docs/demos/aic-scenario.md).

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
