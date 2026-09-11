# ROS Physical AI Demos

Open-source physical-AI applications for ROS 2 — simulation (Gazebo, MuJoCo) and real hardware, with full Record → Train → Deploy learning pipelines powered by [LeRobot](https://github.com/huggingface/lerobot). Pick an application below to get started.

## Applications

This repository hosts several physical-AI applications. Each one is a
self-contained Pixi workspace with its own manifest, lockfile, build tree,
and **ROS 2 distribution** — pick one and work inside it.

| Application | Directory                            | ROS 2        | Simulator               | Status |
| ----------- | ------------------------------------ | ------------ | ----------------------- | ------ |
| SO-ARM101   | [`apps/so_arm101/`](apps/so_arm101/) | Lyrical Luth | Gazebo (conda) / MuJoCo | Active |

```bash
cd apps/so_arm101
pixi run install-deps
pixi run build
pixi run so-arm-gz
```

Shared sources live in [`common/`](common/) and are imported by every
application. The repository root carries no Pixi manifest at all — every
environment belongs to an application.

> **Applications may be on different ROS 2 distributions.** Each application
> under `apps/<name>/` declares its own ROS distribution and dependency
> solve, so two applications may sit on different ROS 2 releases. Always run
> `pixi` commands from inside an application directory; there is no
> environment at the repository root. See the
> [SO-ARM101 Overview](apps/so_arm101/README.md)
> for a worked example.

## Documentation

Each application documents itself under `apps/<name>/docs/`; [`docs/`](docs/README.md)
holds only what applies repository-wide.

| Guide                                          | What it covers                                            |
| ---------------------------------------------- | --------------------------------------------------------- |
| [SO-ARM101 Overview](apps/so_arm101/README.md) | Installation, quick start, packages, hardware, demos, MCP |
| [Contributing](docs/contributing.md)           | Linting and pre-commit hooks                              |

## External demos

Other fully open-source physical AI projects on ROS:

- [Agentic mobile manipulator](https://github.com/RobotecAI/agentic-mobile-manipulator), a comprehensive demo project using a hardware-in-the-loop setup with O3DE and all the software and inference running on-board.
