# End-to-end pipeline on the AIC simulator

Record demonstrations of the cable-insertion task, port them to a LeRobot dataset, train an ACT policy and deploy it against the simulator. Every command is a task in [`pixi.toml`](../../pixi.toml), run from `apps/aic`.

## The contract

[`pai_aic/config/rosetta/aic.yaml`](../../pai_aic/config/rosetta/aic.yaml) maps the simulator's topics to dataset features, at 20 frames per second:

| Feature                     | Topic                           | Content                                                               |
| --------------------------- | ------------------------------- | --------------------------------------------------------------------- |
| `observation.images.left`   | `/left_camera/image`            | 480x480 RGB                                                           |
| `observation.images.center` | `/center_camera/image`          | 480x480 RGB                                                           |
| `observation.images.right`  | `/right_camera/image`           | 480x480 RGB                                                           |
| `observation.state`         | `/joint_states`                 | The six arm joint positions, radians                                  |
| `action`                    | `/aic_controller/pose_commands` | Twist in the tool frame `gripper/tcp`: linear m/s, then angular rad/s |

The action message is AIC's `MotionUpdate`. Two codecs in [`pai_aic/codecs.py`](../../pai_aic/pai_aic/codecs.py) map it to and from the 6-dimensional twist; the encoder also sets velocity mode and the impedance AIC's reference policy commands, so the teleop and the deployed policy send identical messages. The controller spawns no gripper controller and the cable starts attached to the gripper, so there is no gripper dimension.

## Recording

Four terminals, all in `apps/aic`:

```bash
pixi run sim-up-gpu      # 1: the simulator
pixi run record          # 2: the episode recorder
pixi run record-keys     # 3: r starts an episode, s stops it
pixi run teleop          # 4: drive the arm
```

Keys in the teleop terminal: `a/d` and `w/s` and `r/f` move along the tool X, Y and Z axes; `S/W`, `A/D` and `q/e` rotate about them; `k` is slow, `l` is fast; Esc quits. The teleop reads only its own terminal, so switch to terminal 3 to start and stop episodes and to terminal 4 to drive.

Between episodes, reset the scene so every demonstration starts from the same state:

```bash
pixi run reset-scene
```

This homes the arm and spawns a fresh cable in the gripper, at the gripper's live pose, without restarting the simulator. The arm settles a few degrees off the exact home joints under the cable's load; that is the impedance controller's steady state and the same for every episode.

Bags land under `datasets/aic/bags/<timestamp>/`.

## Porting

```bash
pixi run port-bags
```

Writes `datasets/aic_cable_insertion/` with the three video streams, the state, the action and the contract as a sidecar. Pass `--help` after the task name for the porter's options.

## Training

```bash
pixi run train
```

Trains ACT on the ported dataset into `outputs/train/aic_act/`, with the checkpoint at `outputs/train/aic_act/checkpoints/last/pretrained_model`. Extra arguments go through to `lerobot-train`, for example `pixi run train --steps=20000`.

## Deploying

With the simulator up:

```bash
pixi run deploy
```

Starts Rosetta's policy runner with the contract, the parameters in [`pai_aic/config/rosetta/policy_runner.yaml`](../../pai_aic/config/rosetta/policy_runner.yaml) and simulated time, loading the checkpoint above. Then trigger inference from another terminal:

```bash
ros2 action send_goal /run_policy rosetta_interfaces/action/RunPolicy "{prompt: 'insert the cable'}"
```

The arm moves under policy control; cancelling the goal publishes a zero twist. To deploy a different checkpoint, pass `pretrained_name_or_path:=<path>` after the task name.

## Walkthrough

```bash
pixi run install-deps && pixi run build      # once
xhost +local: && pixi run sim-up-gpu         # terminal 1
pixi run record                              # terminal 2
pixi run record-keys                         # terminal 3
pixi run teleop                              # terminal 4
# record several episodes: r, drive, s, then `pixi run reset-scene`
pixi run port-bags
pixi run train
pixi run deploy                              # simulator still up
ros2 action send_goal /run_policy rosetta_interfaces/action/RunPolicy "{prompt: 'insert the cable'}"
```
