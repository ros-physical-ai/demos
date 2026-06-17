# Documentation

Documentation index for the [ROS Physical AI Demos](../README.md) repository.

## Getting started

| Guide                                     | What it covers                                                               |
| ----------------------------------------- | ---------------------------------------------------------------------------- |
| [Installation](installation.md)           | Requirements, Pixi install, manual install, `rmw_zenoh`, ML dependencies     |
| [Development Guide](development.md)       | Pixi-based workflow, building, `pixi shell`, FAQ, and Gazebo troubleshooting |
| [Running the Robot](running-the-robot.md) | Launching the SO-ARM101 in Gazebo, MuJoCo, or on real hardware               |

## Hardware setup

For running on a physical SO-ARM101.

| Guide                                  | What it covers                                                            |
| -------------------------------------- | ------------------------------------------------------------------------- |
| [Calibration](hardware/calibration.md) | Servo calibration via LeRobot, `joint_config_file`, and known limitations |
| [Udev Rules](hardware/udev-rules.md)   | Stable device symlinks for arms and cameras                               |
| [Cameras](hardware/cameras.md)         | Wrist/static camera configuration, calibration files, and TF frames       |

## Teleoperation

| Guide                                      | What it covers                                                           |
| ------------------------------------------ | ------------------------------------------------------------------------ |
| [Teleoperation Overview](teleoperation.md) | Leader-arm, RViz interactive-marker IK, and phone (WebXR) teleop methods |

## Learning pipeline (demos)

| Guide                                                        | What it covers                                   |
| ------------------------------------------------------------ | ------------------------------------------------ |
| [End-to-End Learning Pipeline](demos/end-to-end-pipeline.md) | Record → Train → Deploy with Rosetta and LeRobot |

## Contributing

| Guide                           | What it covers                                           |
| ------------------------------- | -------------------------------------------------------- |
| [Contributing](contributing.md) | Linting and the pre-commit hooks used in this repository |
