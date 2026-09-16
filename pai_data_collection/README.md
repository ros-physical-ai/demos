# pai_data_collection

Data collection tools for Physical AI demos using [rosetta](https://github.com/iblnkn/rosetta).

## Requirements

This project uses [Pixi](https://pixi.sh/) for environment management. Make sure the workspace is set up following the [Development Guide](../docs/development.md).

The required external repos (`rosetta`, `rosetta_interfaces`, `lerobot_rosetta`, and `lerobot_robot_rosetta`) are included in `pai.repos` and will be fetched automatically during workspace setup:

```bash
vcs import external < pai.repos --recursive
```

> [!NOTE]
> The following commands assume you are inside a `pixi shell` session or that you are running via `pixi run`.
> See the [Development Guide](../docs/development.md) for details.

## Recording Rosbag

This package provides a rosetta contract for the SO-ARM101 robot (`config/rosetta/so_arm101.yaml`).
Recording uses rosetta's `episode_recorder_launch.py` directly.

### Workflow

1. Run zenoh router on a separate terminal:

```bash
pixi run zenoh-router # ros2 run rmw_zenoh_cpp rmw_zenohd
```

2. Start simulation:

```bash
pixi run so-arm-gz # ros2 launch pai_bringup so_arm_gz_bringup.launch.py
```

3. Start the episode recorder (using rosetta's launch file with our contract):

```bash
ros2 launch rosetta episode_recorder_launch.py \
    contract_path:=$(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/config/rosetta/so_arm101.yaml \
    bag_base_dir:=datasets/so_arm101/bags \
    use_sim_time:=true
```

> [!NOTE]
> The recorder captures **every topic on the graph** by default. The contract declares which topics must be present for the episode to be valid. Set `record_all:=false` to record only the contract's topics, or trim the capture with the `exclude_topics` regex list in rosetta's `params/episode_recorder.yaml`, if the bags get too large. Episodes are stored as raw messages with the contract text embedded in `metadata.yaml`, so a revised contract can be applied to existing recordings.

4. Start the keyboard controller (in a new terminal):

```bash
ros2 run rosetta episode_keyboard_node
```

Press `t` to set a task prompt, then `r` (or `→`) to start recording.

5. Reset the cubes (Gazebo only, before each episode):

The cubes do not snap back to their starting poses when an episode ends, so reset them before recording the next one. The script lives in this package and is installed automatically by `pixi run build`:

```bash
# Reset to the nominal layout (matches the starting layout in so_arm_table.sdf)
$(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/scripts/gz_set_cubes_poses.py

# Or randomize the pose of each cube within a small region around the nominal
$(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/scripts/gz_set_cubes_poses.py --random --seed 1
```

Run with `--help` to see all options (`--radius`, `--angle-range`, `--pose NAME=...` overrides, `--dry-run`).

6. Move the arm:

You can directly use the forward position controller via topic:

```bash
# Home position (all zeros)
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' --rate 20

# Slight rotation and tilt
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.2, -0.4, 0.0, 0.0, 0.0, 0.4]}' --rate 20
```

There is simple script to run some of these commands sequentially:

```bash
$(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/scripts/arm_demo_positions.sh
```

7. Finish episode: Press `s` (or `←`) to stop and save, or `d` to discard.

This will save a rosbag that corresponds to that episode.

8. Record more episodes: Repeat from step 5 (press `r` again — no need to restart the keyboard controller; reset the cubes to start fresh).

#### Workflow Overview

```mermaid
flowchart LR
    A["1. Start Zenoh Router
    pixi run zenoh-router"] --> B["2. Start Simulation
    pixi run so-arm-gz"]
    B --> C["3. Start Episode Recorder
    ros2 launch rosetta episode_recorder_launch.py ..."]
    C --> D["4. Start Keyboard Controller
    ros2 run rosetta episode_keyboard_node"]
    D --> E["5. Start Episode
    Press r"]
    E --> F["6. Move the Arm
    ros2 topic pub ..."]
    F --> G["7. Finish Episode
    Press s (save) or d (discard)"]
    G --> H{More episodes?}
    H -- Yes --> E
    H -- No --> I["Done Recording Rosbags"]

```

### MuJoCo-based data collection

Use MuJoCo simulation with the same `so_arm101.yaml` contract.

1. Start zenoh router: `pixi run zenoh-router`
2. Start MuJoCo + camera relay: `pixi run so-arm-mujoco`
3. Start rosetta recorder: `pixi run rosetta-record-mujoco`
4. Start keyboard controller (new terminal): `ros2 run rosetta episode_keyboard_node`
5. Press `t` to set a prompt, `r` to start recording
6. Move the arm: `$(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/scripts/arm_demo_positions.sh` (or `ros2 topic pub`; MuJoCo must already be running from step 2)
7. Press `s` to save or `d` to discard the episode

## Convert Rosbag to LeRobot

The contract's action declares an `apply` pipeline that converts ROS radians to LeRobot degrees during conversion, and back to clamped radians when serving actions:

```yaml
# In the contract (config/rosetta/so_arm101.yaml):
actions:
  action:
    channel:
      topic: /forward_position_controller/commands
      type: std_msgs/msg/Float64MultiArray
      safety: hold
    align: { strategy: hold, timeline: receive }
    select: [shoulder_pan.pos, ...]
    apply:
      - clamp: { min: -3.14159, max: 3.14159 }
      - rad2deg
```

> [!IMPORTANT]
> The operator order matters. `apply` runs front-to-back on record and **back-to-front through the inverses** on serve, so `[clamp, rad2deg]` yields `deg2rad` then `clamp` on the way out, bounding the outgoing command in radians. Reversed, it would clamp degrees to ±3.14 and destroy every command.

Run conversion (Gazebo bags use `datasets/so_arm101/bags`; MuJoCo bags use `datasets/so_arm101_mujoco/bags`):

```bash
# Gazebo
ros2 run rosetta rosetta_port \
    --raw-dir datasets/so_arm101/bags \
    --contract $(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/config/rosetta/so_arm101.yaml \
    --repo-id move_arm \
    --root datasets_lerobot

# MuJoCo
ros2 run rosetta rosetta_port \
    --raw-dir datasets/so_arm101_mujoco/bags \
    --contract $(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/config/rosetta/so_arm101.yaml \
    --repo-id move_arm_mujoco \
    --root datasets_lerobot
```

### rosetta_port arguments

`rosetta_port` replaced `python -m rosetta.port_bags` in rosetta 0.2.0. It is invoked through `ros2 run` because rosetta installs its console scripts into `lib/rosetta` rather than onto `PATH`.

| Argument               | Required | Description                                                                   |
| ---------------------- | -------- | ----------------------------------------------------------------------------- |
| `--raw-dir`            | Yes      | Directory containing bag subdirectories (each with `metadata.yaml`)           |
| `--contract`           | Yes      | Path to rosetta contract YAML                                                 |
| `--repo-id`            | No       | Dataset name. Defaults to `--raw-dir` directory name                          |
| `--root`               | No       | Parent directory for datasets. Dataset saved to `root/repo-id`                |
| `--framework`          | No       | Learning framework to write for (default: `lerobot`, resolved by entry point) |
| `--push-to-hub`        | No       | Upload to HuggingFace Hub after conversion                                    |
| `--vcodec`             | No       | Video codec (default: `libsvtav1`). Use `libx264` for faster encoding         |
| `--streaming-encoding` | No       | Encode frames directly instead of via intermediate PNGs (faster)              |
| `--no-embed-contract`  | No       | Skip the `meta/rosetta_contract.yaml` sidecar                                 |

## Replay Dataset on Real Robot using LeRobot

Using local LeRobot dataset (from within the pixi environment):

```bash
lerobot-replay \
    --robot.type=so101_follower \
    --robot.port=/dev/so101_follower \
    --robot.id=my_awesome_arm \
    --dataset.repo_id=move_arm \
    --dataset.root=datasets_lerobot/move_arm \
    --dataset.episode=0 \
    --robot.use_degrees=true \
    --play_sounds=false
```

**Important flags:**

- `--robot.use_degrees=true` - Required because the dataset contains degree values (from `apply: [rad2deg]` in the contract)
- `--play_sounds=false` - Disable audio feedback (avoids `spd-say` errors)
