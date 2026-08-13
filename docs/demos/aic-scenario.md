# AIC Cable-Insertion Scenario (via `pai_aic`)

This guide walks you through demos' Record → Train → Deploy pipeline applied to
the [AI for Industry Challenge](https://github.com/intrinsic-dev/aic)
cable-insertion scenario — a UR5 arm inserting cables into a NIC/SC port board
in a Gazebo sim, under cartesian impedance control.

The integration is implemented by the [`pai_aic`](../../pai_aic/README.md)
package: a small `ament_python` package that adds a custom Rosetta contract,
three converter functions (state ↔ Observation.msg, action ↔ MotionUpdate), a
generic `LerobotPolicy` Policy class, and two thin launch wrappers. **No
upstream code in `external/rosetta`, `external/lerobot-robot-rosetta`, or
`external_aic/aic` is modified.**

```
  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
  │ AIC BRINGUP   │    │ RECORD   │    │ CONVERT  │    │  TRAIN   │    │ DEPLOY   │
  │ UR5 + aic_   │───▶│ via      │───▶│ port_bags│───▶│ lerobot- │───▶│ Main:    │
  │ controller   │    │ demos'   │    │ (custom  │    │ train    │    │ rosetta_ │
  │ + aic_adapter│    │ episode_ │    │ decoder) │    │          │    │ client   │
  │ (URDF +      │    │ recorder │    │          │    │          │    │ (async)  │
  │ 3 cameras)   │    │          │    │          │    │          │    │          │
  └──────────────┘    └──────────┘    └──────────┘    │          │    │ Alt:     │
                                                       │          │    │ aic_model│
                                                       │          │    │ +Lerobot │
                                                       └──────────┘    └──────────┘
```

> [!TIP]
> The same LeRobot-trained checkpoint deploys via **two paths** sharing one
> contract and one converter set. This guide **prioritizes the Rosetta path**
> (demos-native) and treats the `aic_model` path as an alternative:
> - **Recommended — Rosetta** (`aic-deploy-rosetta`) — async inference via
>   demos' `rosetta_client_node` with chunked overlap. The default throughout
>   this guide.
> - **Alternative — `aic_model`** (`aic-deploy-aic-model`) — synchronous
>   inference inside `aic_model`, native to AIC's lifecycle and required for
>   `aic_engine` evaluation. If you want the full AIC-native flow, you can also
>   go straight to [AIC upstream](https://github.com/intrinsic-dev/aic).

## Prerequisites

Follow the main [README](../../README.md) to set up the workspace, then add the
AIC demo's sources and build them into the `aic` environment:

```bash
git clone https://github.com/ros-physical-ai/demos && cd demos
pixi install
pixi run setup-aic          # import pai.repos + aic.repos (UR5, gz-from-source, ros-controls, aic packages)
pixi run install-ml-deps    # PyTorch + LeRobot (auto-detects GPU)
pixi run aic-build          # build the AIC demo (from-source Gazebo) into install_aic/
```

> [!NOTE]
> All commands below assume you are inside a `pixi shell -e aic` session or
> running via `pixi run` (the `aic-*` tasks select the `aic` environment
> automatically).

> [!IMPORTANT]
> The AIC demo lives in its own `aic` Pixi environment (from-source Gazebo)
> and its own `install_aic/` build tree, kept separate from the base
> SO-ARM101 demo so the two never interfere. `pixi run setup-aic` imports
> **both** `pai.repos` and `aic.repos`: `aic.repos` brings in the `aic`
> packages themselves (`aic_bringup`, `aic_engine`, `aic_model`, `aic_adapter`,
> `aic_controller`, …) under `external_aic/aic/`, **and** their build dependencies
> (the Universal Robots UR5 description/driver, the from-source Gazebo stack,
> and from-source `ros2_control` / `ros2_controllers`) under
> `external_aic/aic_repos/`. Skipping the `aic.repos` import leaves `aic_bringup`
> unbuildable.

Additional requirements for this scenario:

| Requirement | Notes |
|---|---|
| **NVIDIA GPU** | Recommended for training and inference. Use `--policy.device=cpu` if absent (inference will be much slower). |
| **Internet access** | `lerobot-train` downloads pretrained stats from the HuggingFace Hub on first run. |
| **Gazebo sim** | The AIC bringup launches Gazebo Harmonic with the UR5 + task board. |
| **AIC packages** | Brought in via `aic.repos` under `external_aic/aic/`. ROS-deps in the `aic` Pixi feature. |
| **AIC build deps** | Brought in via `aic.repos` under `external_aic/aic_repos/` (UR5 description/driver, from-source Gazebo, from-source `ros2_control`/`ros2_controllers`). |

> [!IMPORTANT]
> This project uses `rmw_zenoh` as the ROS 2 middleware. The Zenoh router must
> be running before launching any demo. Start it in a dedicated terminal and
> leave it running for the duration of your session:
>
> ```bash
> pixi run start_zenoh
> ```
>
> See [Running the Robot](../running-the-robot.md) for details.

### What `pai_aic` adds to your workspace

| Path | Purpose |
|---|---|
| `pai_data_collection/config/rosetta/aic.yaml` | The contract — 3 cameras + 26-dim state + 6-dim action |
| `pai_aic/pai_aic/converters.py` | Custom decoders/encoder (`decode_aic_observation`, `encode_aic_motion_update`, `decode_aic_motion_update`) |
| `pai_aic/pai_aic/policies/LerobotPolicy.py` | The `aic_model` Policy class for the alternative `aic_model` deploy path |
| `pai_aic/scripts/aic_spawn_scene.py` | Standalone scene spawner (task board + cable) for recording without `aic_engine` |
| `pai_aic/config/scene_example.yaml` | Example scene definition consumed by `aic_spawn_scene.py` (mirrors `trial_1`) |
| `pai_aic/launch/aic_record.launch.py` | Wraps `rosetta/episode_recorder_launch.py` |
| `pai_aic/launch/aic_deploy_rosetta.launch.py` | Wraps `rosetta/rosetta_client_launch.py` (recommended Rosetta deploy) |
| `pixi.toml` | 7 new tasks: `aic-gz`, `aic-spawn-scene`, `aic-engine`, `aic-record`, `aic-train`, `aic-deploy-aic-model`, `aic-deploy-rosetta` |

---

## The Pipeline

### 1. The Contract

The contract at `pai_data_collection/config/rosetta/aic.yaml` declares:

| ROS 2 Side | | LeRobot Side |
|---|---|---|
| `/left_camera/image` (`sensor_msgs/Image`) | → | `observation.images.left_camera` |
| `/center_camera/image` (`sensor_msgs/Image`) | → | `observation.images.center_camera` |
| `/right_camera/image` (`sensor_msgs/Image`) | → | `observation.images.right_camera` |
| `/observations` (`aic_model_interfaces/Observation`) | → | `observation.state` (26-dim, via custom decoder) |
| `/aic_controller/pose_commands` (`MotionUpdate`) | ← | `action` (6-dim, via custom encoder/decoder) |

Key contract features:

- **FPS**: 20 Hz (matches `aic_adapter` publish rate — matches AIC's reference)
- **State layout**: 26-dim, raw SI units (meters, radians).
- **Action layout**: 6-dim cartesian twist (linear xyz + angular xyz).
- **Image resize**: 480×480 for neural network input.
- **Unit conversion**: none on state or action (AIC works in raw SI; the AIC reference policy was trained in radians).
- **No `rad2deg`** — unlike demos' SO-ARM101 contract, the AIC impedance controller works in SI units.
- **Action safety behavior**: `hold` — at inference stop, the last commanded twist is held by the impedance controller.

> [!NOTE]
> The cameras in the contract are subscribed as raw `/left_camera/image`,
> `/center_camera/image`, `/right_camera/image` from the `ros_gz_bridge`, **not**
> from inside `aic_model_interfaces/msg/Observation`. This lets the standard
> `sensor_msgs/Image` decoder handle bgr8/rgb8 conversion and resize. Only the
> state is decoded from the fused `Observation.msg`.

### 2. Bring up the AIC sim

The AIC bringup (UR5 + impedance controller + sensor-fusion adapter) launches
on its own; demos does not add or wrap `aic_bringup`.

```bash
# Terminal 2 — AIC sim (no engine during recording or free-running deploy)
pixi run aic-gz
```

This calls:

```bash
ros2 launch aic_bringup aic_gz_bringup.launch.py start_aic_engine:=false
```

When the bringup is healthy you should see (in `ros2 topic list`):

```
/left_camera/image           /center_camera/image        /right_camera/image
/left_camera/camera_info     /center_camera/camera_info  /right_camera/camera_info
/joint_states
/observations                aic_model_interfaces/msg/Observation
/aic_controller/pose_commands  aic_control_interfaces/msg/MotionUpdate
/aic_controller/joint_commands  aic_control_interfaces/msg/JointMotionUpdate
/change_target_mode            aic_control_interfaces/srv/ChangeTargetMode
```

> [!IMPORTANT]
> The `/observations` topic is published by `aic_adapter`, which the bringup
> starts by default. The contract subscribes to `/observations` directly. If
> `/observations` is missing, recording/deploy will fail with no state stream —
> check that `aic_adapter` is running (`ros2 node list | grep aic_adapter`).

### 3. Recording Episodes

> [!IMPORTANT]
> **Do not use `aic_engine` for teleoperated recording.** The engine runs its
> own autonomous trial loop — it spawns a scene, waits for a *participant
> model* to drive the arm, scores the attempt via `/scoring/insertion_event`,
> then resets and advances to the next trial. That loop is built for
> **evaluation of a trained policy**, not for a human teleoperating the arm to
> collect demonstrations. For recording, spawn the scene yourself with
> `pixi run aic-spawn-scene` and manage resets between episodes manually.
> `aic_engine` comes back in at
> [Evaluating with `aic_engine`](#evaluating-with-aic_engine).

> [!TIP]
> **`aic-spawn-scene` owns the scene while you record.** It spawns the full
> configured scene (task board + per-rail cards/ports + cable) from a YAML
> definition, and re-spawns it between episodes with `--clear`. The typical
> recording setup is:
>
> 1. Terminal 1 — Zenoh router (`pixi run start_zenoh`).
> 2. Terminal 2 — AIC sim without engine (`pixi run aic-gz`).
> 3. Terminal 2.5 — **`pixi run aic-spawn-scene`** (spawns the populated
>    scene; re-run with `-- --clear` to reset between episodes).
> 4. Terminal 3 — Recorder (`pixi run aic-record`).
> 5. Terminal 4 — Teleop (see below).
> 6. Terminal 5 — `episode_keyboard_node` to start/stop rosbag recording.

For data collection you:

1. Start the Zenoh router (Terminal 1).
2. Start the AIC sim without the engine (Terminal 2). `pixi run aic-gz`
   brings up the UR5, impedance controller, and `aic_adapter` but does
   **not** spawn the task board, ports, or cable.
3. Spawn the populated scene (Terminal 2.5). `pixi run aic-spawn-scene`
   reads a scene YAML and spawns the task board + per-rail cards/ports
   + cable. Re-run with `-- --clear` to reset between episodes.
4. Start the episode recorder (Terminal 3).
5. Drive the arm via teleop (Terminal 4 — see below).
6. Use `episode_keyboard_node` to start/stop recordings (Terminal 5).

```bash
# Terminal 2 — Sim (no engine)
pixi run aic-gz

# Terminal 2.5 — Spawn the populated scene (task board + cable)
pixi run aic-spawn-scene
# Re-run with --clear between episodes to reset the scene:
#   pixi run aic-spawn-scene -- --clear

# Terminal 3 — Recorder
pixi run aic-record
```

This wraps `rosetta/episode_recorder_launch.py` with the AIC contract and
defaults `bag_base_dir:=$HOME/datasets/aic/bags`. Override either:

```bash
ros2 launch pai_aic aic_record.launch.py \
    bag_base_dir:=/scratch/aic_episodes \
    use_sim_time:=true
```

```bash
# Terminal 4 — Keyboard controller (demos' episode_keyboard_node)
ros2 run rosetta episode_keyboard_node
```

| Key | Action |
|---|---|
| `r` / `→` | Start recording |
| `s` / `←` | Stop and save (writes a rosbag subdir under `bag_base_dir`) |
| `d` / `⌫` | Discard episode (stop + delete bag) |
| `t` | Edit task prompt for the next episode |

#### Teleop sources

Pick **one** of:

| Source | How to launch | Notes |
|---|---|---|
| **LeRobot `aic_keyboard_ee`** (recommended for real demos) | `pixi run lerobot-teleoperate --robot.type=aic_controller --robot.id=aic --teleop.type=aic_keyboard_ee --teleop.id=aic --robot.teleop_target_mode=cartesian --robot.teleop_frame_id=base_link --display_data=false` | Rich keyboard control with speed modes, shift-modifier for rotation. |
| **Scripted `ros2 topic pub`** (smoke tests / CI) | `ros2 topic pub /aic_controller/pose_commands aic_control_interfaces/msg/MotionUpdate '{...}' --rate 20` | Stand-in for end-to-end infra validation. No UI deps. |

Whichever you pick, it publishes `MotionUpdate` on
`/aic_controller/pose_commands` — exactly the topic the contract records. The
recorder captures these as raw `MotionUpdate` rosbag streams and `port_bags`
decodes them into 6-dim action vectors at convert time.

#### Resetting the scene between episodes

Because you own the scene lifecycle during recording, you reset it yourself
between takes. The `aic-spawn-scene` helper is the tool for this.

##### `pixi run aic-spawn-scene` (the recording path)

You brought up the sim without `aic_engine` (`pixi run aic-gz`, no engine),
so you drive the scene lifecycle yourself. The `aic-spawn-scene` helper
spawns the **full populated scene** (task board with its per-rail
cards/ports/mounts **plus** the cable) from a YAML definition, and can
re-spawn it between episodes with `--clear`.

The helper reads a scene YAML that uses the **exact same schema** as
the `trials.<id>.scene` block of the `aic_engine` config, so you can
copy sections between them or point `--scene-file` straight at the
engine's `sample_config.yaml` with `--trial <id>`. It ships with
`pai_aic/config/scene_example.yaml` (mirrors `trial_1`) as the default.

Per invocation it:

1. (with `--clear`) tears down `task_board` and the scene's cables
   (idempotent).
2. Spawns the **task board** as a single Gazebo entity — the URDF
   includes the board base plus every per-rail card/port/mount whose
   `entity_present: true` in the YAML (`nic_card_mount_0`, `sc_port_0`,
   `sfp_mount_0`, etc.), with translations clamped to the rail limits.
3. Spawns each **cable** at `world → gripper/tcp` + `gripper_offset`
   (so the robot must already be up).

Run it via the `pixi run aic-spawn-scene` task — it activates the
ROS + Gazebo env for you. **Pass helper flags after `--`**:

```bash
# Spawn the default populated scene (scene_example.yaml).
pixi run aic-spawn-scene

# Spawn with ground-truth TF frames (useful while recording).
pixi run aic-spawn-scene -- --ground-truth

# Reset between episodes: tear down + rebuild the whole scene.
pixi run aic-spawn-scene -- --clear --ground-truth

# Use your own scene file, or an engine trial directly.
pixi run aic-spawn-scene -- --scene-file /path/to/your_scene.yaml
pixi run aic-spawn-scene -- \
    --scene-file $(ros2 pkg prefix aic_engine)/share/aic_engine/config/sample_config.yaml \
    --trial trial_2

# Spawn only part of the scene.
pixi run aic-spawn-scene -- --no-cables      # task board only
pixi run aic-spawn-scene -- --no-task-board  # cables only
```

If you're already inside `pixi shell`, you can call the script
directly without the `pixi run` prefix:

```bash
python "$(ros2 pkg prefix pai_aic)/share/pai_aic/scripts/aic_spawn_scene.py" --clear --ground-truth
```

Full helper reference (`pixi run aic-spawn-scene -- --help`):

```
--scene-file PATH        (default: pai_aic/config/scene_example.yaml)
--trial ID               (select a trial when the file has a 'trials' block)
--ground-truth           (expand the task board with ground-truth TF frames)
--clear                  (delete task board + cables before spawning)
--no-task-board          (skip spawning the task board)
--no-cables              (skip spawning cables)
--gripper-frame FRAME    (default 'gripper/tcp')
```

> [!TIP]
> `attach_cable_to_gripper: true` in the YAML spawns the cable already
> attached to the gripper — use it when the policy starts with the
> cable held. For data collection demos where the policy will **pick
> up** the cable, set it to `false` (cable sits free on the board).

##### Manual one-liners (spawn/reset by hand)

If you'd rather run the primitives by hand (e.g. for debugging), the
equivalent low-level steps are:

```bash
# 1. Tear down the cable (idempotent) and re-spawn it at the nominal pose.
gz service -s /world/aic_world/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --req "name: 'cable_0'" || true
ros2 launch aic_bringup spawn_cable.launch.py

# 2. Reset the arm joints to the home pose.
ros2 service call /aic_controller/reset_joints aic_engine_interfaces/srv/ResetJoints \
  --field joint_names "['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']" \
  --field initial_positions "[0.6, -1.5708, -1.5708, -1.5708, 1.5708, 0.6]"
```

> [!NOTE]
> The bare `spawn_cable.launch.py` / `spawn_task_board.launch.py` from
> `aic_bringup` only spawn a cable / an **empty** board (all per-rail
> `_present:=false`). For a populated board without `aic_engine`, use
> `aic-spawn-scene` above — it builds the single `xacro → spawn` call
> with the per-rail args derived from your scene YAML.

### 4. Converting to LeRobot Dataset

After recording, convert the rosbags to a LeRobot dataset:

```bash
python -m rosetta.port_bags \
    --raw-dir $HOME/datasets/aic/bags \
    --contract $(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/config/rosetta/aic.yaml \
    --repo-id aic_cable_insertion \
    --root $HOME/datasets
```

The contract's custom decoder (`pai_aic.converters:decode_aic_observation`) and
inverse encoder (`decode_aic_motion_update`) are applied automatically. The
dataset's `meta/info.json` will record:

| Feature | dtype | Shape |
|---|---|---|
| `observation.images.left_camera` | video | — |
| `observation.images.center_camera` | video | — |
| `observation.images.right_camera` | video | — |
| `observation.state` | float64 | (26,) |
| `action` | float64 | (6,) |

> [!TIP]
> Add `--push-to-hub` (with a namespaced `--repo-id`, e.g. `your-hf-user/aic_cable_insertion`) to upload to the
> HuggingFace Hub in one step. Make sure you are logged in first (`hf auth login`).

### 5. Training a Policy

Train any LeRobot-supported policy on the converted dataset:

```bash
pixi run aic-train
```

Which expands to:

```bash
lerobot-train \
    --dataset.repo_id=aic_cable_insertion \
    --dataset.root=$HOME/datasets/aic_cable_insertion \
    --policy.type=act \
    --output_dir=outputs/train/aic_act \
    --job_name=aic_act \
    --policy.device=cuda
```

For a quick smoke test (10 steps, CPU-only):

```bash
lerobot-train \
    --dataset.repo_id=aic_cable_insertion \
    --dataset.root=$HOME/datasets/aic_cable_insertion \
    --policy.type=act \
    --output_dir=outputs/train/aic_act_smoke \
    --policy.device=cpu \
    --batch_size=2 \
    --steps=10 \
    --eval_freq=1000 \
    --save_freq=10
```

The checkpoint ends up at `outputs/train/aic_act/checkpoints/last/pretrained_model/`.

> [!NOTE]
> The `pixi run aic-train` task is a sensible default but is intentionally
> short — adjust `--steps`, `--batch_size`, and `--policy.device` to your
> hardware before serious training. To resume from a checkpoint:
>
> ```bash
> lerobot-train \
>     --config_path=outputs/train/aic_act/checkpoints/last/pretrained_model/train_config.json \
>     --resume=true
> ```

### 6. Deploying a Policy

Two deploy paths share the **same** checkpoint and **same** contract. This guide
**leads with the Rosetta path** (demos-native) and keeps `aic_model` as an
alternative for AIC-native / `aic_engine` use.

#### Recommended — `rosetta_client_node` (demos-native, async inference)

```bash
# Terminal 1 — Zenoh
pixi run start_zenoh

# Terminal 2 — AIC sim
pixi run aic-gz

# Terminal 3 — rosetta_client_node with the AIC contract
pixi run aic-deploy-rosetta
```

The pixi task runs:

```bash
ros2 launch pai_aic aic_deploy_rosetta.launch.py \
    pretrained_name_or_path:=outputs/train/aic_act/checkpoints/last/pretrained_model
```

What you should see:

1. `rosetta_client` logs `Node created (unconfigured)` then `active`.
2. A policy server subprocess is launched automatically (visible in `ps aux | grep policy_server`).
3. Action rate:
   ```bash
   ros2 topic hz /aic_controller/pose_commands
   # → ~50 Hz with chunked overlap (rosetta_client_node's default)
   ```

> [!NOTE]
> `aic_deploy_rosetta.launch.py` wraps
> `rosetta/rosetta_client_launch.py` with the AIC contract. To override
> defaults, pass launch args directly:
>
> ```bash
> ros2 launch pai_aic aic_deploy_rosetta.launch.py \
>     pretrained_name_or_path:=/path/to/checkpoint \
>     policy_type:=act \
>     policy_device:=cpu \
>     use_sim_time:=true
> ```

#### Alternative — `aic_model` + `LerobotPolicy` (AIC-native, for `aic_engine`)

Use this path when you want the AIC-native lifecycle or plan to evaluate with
`aic_engine` (see [Evaluating with `aic_engine`](#evaluating-with-aic_engine)).
For the full AIC-native flow you can also go directly to
[AIC upstream](https://github.com/intrinsic-dev/aic).

```bash
# Terminal 1 — Zenoh
pixi run start_zenoh

# Terminal 2 — AIC sim (no engine)
pixi run aic-gz

# Terminal 3 — Policy via aic_model + LerobotPolicy
pixi run aic-deploy-aic-model
```

The pixi task runs:

```bash
ros2 run aic_model aic_model --ros-args \
    -p use_sim_time:=true \
    -p policy:=pai_aic.policies.LerobotPolicy \
    -p checkpoint_path:=outputs/train/aic_act/checkpoints/last/pretrained_model \
    -p policy_type:=act
```

What you should see:

1. `aic_model` logs `Loaded policy module pai_aic.policies.LerobotPolicy`.
2. `LerobotPolicy` logs `LerobotPolicy ready: type=act checkpoint=... device=... loop_rate_hz=20.0`.
3. The lifecycle enters `active`. Check:
   ```bash
   ros2 lifecycle get /aic_model
   # → active [3]
   ```
4. Action rate on `/aic_controller/pose_commands`:
   ```bash
   ros2 topic hz /aic_controller/pose_commands
   # → ~20 Hz (synchronous inference + execution)
   ```

**This path is `aic_engine`-compatible.** To run a full engine validation pass, see
[`aic_engine/README.md`](../../external_aic/aic/aic_engine/README.md) for a trial
config. Launch the engine with `start_aic_engine:=true` and the same
`LerobotPolicy` should satisfy the engine's lifecycle requirements
(`InsertCable` action server, `/cancel_task` service, rejection-of-goals-when-not-active).

#### Comparing the two paths

| | Recommended (Rosetta) | Alternative (`aic_model`) |
|---|---|---|
| Lifecycle node | `rosetta_client` | `aic_model` |
| Inference | async chunked via `RobotClient` | sync loop in-process |
| Inference ↔ execution overlap | yes (queue of action chunks) | none |
| Control rate | ~50 Hz nominal | ~20 Hz |
| aic_engine validation | ❌ no `InsertCable` action | ✅ native (`InsertCable` action exposed) |
| Requires GPU subprocess | yes (policy server) | no |

### Evaluating with `aic_engine` (alternative)

> [!NOTE]
> `aic_engine` is an **alternative, AIC-native evaluation flow** — not the
> primary path of this guide, which centers on Rosetta. It only works with the
> `aic_model` deploy path (`aic-deploy-aic-model`). If you want to lean fully
> into the AIC engine, you can also use it directly from
> [AIC upstream](https://github.com/intrinsic-dev/aic).

When you launch the AIC sim with the engine enabled, the engine **owns**
spawning and cleanup of task boards (and optionally cables) per trial. You
configure this with a trial YAML.

```bash
# Terminal 1 — Zenoh
pixi run start_zenoh

# Terminal 2 — AIC sim WITH engine, pointing at a trial config
ros2 launch aic_bringup aic_gz_bringup.launch.py \
    start_aic_engine:=true \
    config_file_path:=$(ros2 pkg prefix aic_engine)/share/aic_engine/config/sample_config.yaml

# Terminal 3 — aic_model + LerobotPolicy (the aic_model path — engine requires the InsertCable action server)
pixi run aic-deploy-aic-model
```

> [!NOTE]
> Only the `aic_model` path (`aic-deploy-aic-model`) is compatible with
> `aic_engine`. The Rosetta path (`aic-deploy-rosetta`) does not expose the
> `InsertCable` action server, so the engine rejects its goals — keep
> `start_aic_engine:=false` when using Rosetta.

See [`aic_engine/README.md`](../../external_aic/aic/aic_engine/README.md) and the
sample configs in `external_aic/aic/aic_engine/config/` for how to author
multi-trial evaluation batches with randomized board poses. When the engine is
running, **do not** use the manual reset commands from
[Resetting the scene between episodes](#resetting-the-scene-between-episodes) —
the engine handles spawn/destroy itself.

---

## See also

- [`pai_aic/README.md`](../../pai_aic/README.md) — quick reference for the package
- [End-to-End Learning Pipeline](end-to-end-pipeline.md) — SO-ARM101 equivalent
- [Try a Pre-trained Policy](pretrained-demo.md) — skip-the-training shortcut (SO-ARM101)
- [End-to-end-pipeline doc updates](end-to-end-pipeline.md#aic-cable-insertion-scenario-via-pai_aic) — the AIC subsection added to the main pipeline guide
- AIC upstream: <https://github.com/intrinsic-dev/aic>
