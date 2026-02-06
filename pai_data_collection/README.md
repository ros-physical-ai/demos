# pai_data_collection

Data collection tools for Physical AI demos using [rosetta](https://github.com/iblnkn/rosetta).

## Requirements

This project uses [Pixi](https://pixi.sh/) for environment management. Make sure the workspace is set up following the [Development Guide](../docs/DEVELOPMENT.md).

The required repos (`rosetta` and `rosetta_interfaces`) are included in `pai.repos` and will be fetched automatically during workspace setup:
```bash
vcs import repos < pai.repos --recursive
```

> [!NOTE]
> The following commands assume you are inside a `pixi shell` session or that you are running via `pixi run`.
>  See the [Development Guide](../docs/DEVELOPMENT.md) for details.

## Recording Rosbag

### Workflow

1. Start simulation:
```bash
ros2 launch pai_bringup so_arm_gz_bringup.launch.py
```

2. Start recording:
```bash
ros2 launch pai_data_collection so_arm_record.launch.py bag_base_dir:=datasets/so_arm100/bags
```

3. Start episode:
```bash
ros2 action send_goal /record_episode rosetta_interfaces/action/RecordEpisode "{prompt: 'move arm'}"
```

4. Move the arm:

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

5. Finish episode:
```bash
ros2 service call /record_episode/cancel std_srvs/srv/Trigger "{}"
```

This will save a rosbag that corresponds to that episode.

6. Record more episodes: Repeat steps 3, 4, 5.

## Convert Rosbag to LeRobot

The contract (`so_arm100.yaml`) specifies `unit_conversion: rad2deg` in the action's `from_tensor` section, which automatically converts ROS radians to LeRobot degrees during conversion.

```yaml
# In the contract (config/rosetta/so_arm100.yaml):
actions:
  - key: action
    ...
    from_tensor:
      clamp: [-3.14159, 3.14159]
      unit_conversion: rad2deg  # Converts radians → degrees for LeRobot
```

Run conversion:
```bash
python3 repos/rosetta/scripts/bag_to_lerobot.py \
    --out datasets_lerobot/move_arm \
    --contract=$(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/config/rosetta/so_arm100.yaml \
    --bags datasets/so_arm100/bags/<episode_dir1>/ datasets/so_arm100/bags/<episode_dir2>/
```

## Replay Dataset on Real Robot using LeRobot

Using local LeRobot dataset (from within the pixi environment):
```bash
lerobot-replay \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=my_awesome_arm \
    --dataset.repo_id=move_arm \
    --dataset.root=/ros_ws/src/datasets_lerobot/move_arm \
    --dataset.episode=0 \
    --robot.use_degrees=true \
    --play_sounds=false
```

**Important flags:**
- `--robot.use_degrees=true` - Required because the dataset contains degree values (from `unit_conversion: rad2deg` in the contract)
- `--play_sounds=false` - Disable audio feedback (avoids `spd-say` errors)
