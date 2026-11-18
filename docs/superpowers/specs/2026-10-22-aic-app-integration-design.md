# Design: `apps/aic` — the Record → Train → Deploy pipeline on the AIC cable-insertion simulator

- Date: 2026-10-22
- Status: Approved (grilling session) — pending spec review
- Author: Franco Cipollone (with Claude Fable 5.1)
- Supersedes: the `apps/aic` sections (2, 4, 5, 6, 7) of
  `2026-09-11-multi-app-repo-structure-design.md` and the whole of
  `../plans/2026-09-11-aic-app-docker-first.md`. The repository-structure
  decisions of the September spec (sections 1 and 3) stay in force.
- Also supersedes: `2026-06-30-pai-aic-integration-design.md`.
- Vocabulary: [`GLOSSARY.md`](../../../GLOSSARY.md). Terms in **bold** on
  first use are defined there.

## Goal

Add a second **application**, `apps/aic/`, that runs this repository's
**pipeline** against the AIC cable-insertion scenario: a UR5e under Cartesian
impedance control inserting a fiber cable into a task board in Gazebo. The
simulator comes from the container image AIC publishes. Recording, porting,
training and deployment are the same Rosetta and LeRobot tools the SO-ARM101
application uses.

## Context

### What the September spec settled and this spec keeps

- One Pixi workspace per application under `apps/<name>/`, each with its own
  manifest, lockfile, ROS 2 distribution and build tree. No manifest at the
  repository root.
- `common/common.repos` pins the Rosetta stack; every application vcs-imports
  it into its own `external/` and compiles it into its own `install/`.
- Shared `pai_*` packages are promoted to `common/` on the second consumer.
- The simulator runs in the published `aic_eval` image. Everything of ours
  runs on the host in `apps/aic`'s Pixi environment.
- `apps/aic` is ROS 2 Kilted on Python 3.12 to match the image. The link
  between host and container is `rmw_zenoh_cpp` peers against the router the
  image starts on port 7447. Kilted ships `rmw_zenoh_cpp` 0.6.6; Lyrical ships
  0.10.4. Matching the container removes the interoperability question.
- The image is pinned by digest, the AIC source checkout by SHA, and a drift
  guard in CI proves the two agree on the message definitions the host uses.

### What changed since September

| Area                 | September                                         | Today                                                                                                                                                                                              |
| -------------------- | ------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Rosetta              | 0.1.x contract schema, `rosetta_client_node`      | 0.2.0: schema v2 (`channel:` block, directional `apply`), `policy_runner_node`, codecs referenced by `module:function` path or entry point, at most one numeric observation key and one action key |
| LeRobot              | 0.5.1                                             | 0.6.0, mandatory for Rosetta 0.2.0                                                                                                                                                                 |
| Repository           | single workspace                                  | `apps/so_arm101/` + `common/` on branch `francocipollone/reorganization`                                                                                                                           |
| AIC upstream         | main at a June checkout                           | main `6546127` (2026-10-01). Three commits since June: docs, a camera-mount STL, `pixi.lock`, nine lines of `aic.repos`. Interfaces, controller, adapter, bringup and Dockerfile are unchanged     |
| `aic_eval` image     | index digest `9aa2ffdb…` (`latest` on 2026-09-11) | `latest` still resolves to the same index digest                                                                                                                                                   |
| Old `pai_aic` branch | source for the port                               | contract is schema v1, converters target `rosetta.common.converters` (gone), `LerobotPolicy.py` hand-loads ACT with LeRobot 0.5 internals                                                          |

### AIC facts the design rests on

- Robot: UR5e, Robotiq Hand-E gripper, ATI Axia80 force/torque sensor, three
  Basler wrist cameras at 1152x1024, 20 FPS.
- Controllers (500 Hz `controller_manager`): `joint_state_broadcaster`,
  `fts_broadcaster`, `aic_controller` (type `aic/Controller`, effort-based
  Cartesian impedance). No gripper controller is spawned. The cable starts
  attached to the gripper; the gripper opening is fixed at spawn.
- Command topic: `/aic_controller/pose_commands`,
  `aic_control_interfaces/msg/MotionUpdate`. In `MODE_VELOCITY` the
  controller follows `velocity` (a `Twist`) in `header.frame_id`, which is
  `base_link` or `gripper/tcp`. Stiffness and damping are 6x6 matrices in the
  same message. The controller expects commands at 10 to 30 Hz.
- Observation topics the host uses: `/joint_states`
  (`sensor_msgs/msg/JointState`, arm joints `shoulder_pan_joint`,
  `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`,
  `wrist_3_joint`, plus the gripper finger joints) and
  `/{left,center,right}_camera/image` (`sensor_msgs/msg/Image`).
- Scene: `aic_gz_bringup.launch.py` accepts the task board pose, per-rail
  mounts, cable type and pose, and `attach_cable_to_gripper`.
  `spawn_cable.launch.py` spawns a cable on its own. The
  `ResetJointsPlugin` serves `/scoring/reset_joints`
  (`aic_engine_interfaces/srv/ResetJoints`, joint names plus positions).
  `aic_engine`'s sample config defines the home joint positions.
- Image entrypoint: sources `/ws_aic/install`, exports
  `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, starts `rmw_zenohd` listening on
  `tcp/[::]:7447` with shared memory disabled, then `exec`s
  `ros2 launch aic_bringup aic_gz_bringup.launch.py "$@"`. Zenoh ACL is off
  unless `AIC_ENABLE_ACL` is set.
- AIC's keyboard teleop (`aic_teleoperation`) listens through `pynput`, which
  on X11 receives every keystroke on the desktop regardless of focus.
  Rosetta's `episode_keyboard_node` reads its own terminal in raw mode.
  Their key sets overlap (`r`, `s`, `d`, `t`).

## Decisions

### 1. Process boundary

```
┌─ simulator container (network_mode: host, ROS 2 Kilted) ──────────────┐
│  gz sim │ aic_controller │ aic_adapter │ ros_gz_bridge │ [aic_engine]  │
│  rmw_zenohd  ──►  :7447                                               │
└────────────────────────────┬──────────────────────────────────────────┘
                             │  /joint_states, /{left,center,right}_camera/image
                             │  /aic_controller/pose_commands, /clock
┌────────────────────────────┴─ host: apps/aic Pixi env (Kilted, py312) ─┐
│  pai_aic teleop     episode_recorder_node + episode_keyboard_node      │
│  rosetta_port  ──►  lerobot-train  ──►  policy_runner_node             │
└───────────────────────────────────────────────────────────────────────┘
```

The container runs only what AIC publishes. The host runs every process of
ours. The two sides share nothing but ROS 2 topics through the container's
router. `docker compose exec` into the container is used for one thing only:
**scene reset** (decision 6).

Host-side settings, all set by the application's environment so nothing
depends on the user's shell:

- `ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'`, exported
  by `scripts/activate.sh`. The container's router has shared memory off and
  the two sides do not share `/dev/shm`.
- `ZENOH_SESSION_CONFIG_URI` pointing at `config/zenoh/session.json5`, a
  copy of `rmw_zenoh_cpp`'s default session config with the
  `transport/shared_memory/messages` field removed. The robostack-kilted
  `rmw_zenoh_cpp` 0.6.9 build ships that field while linking `libzenohc`
  1.9, which rejects it, so a session cannot be created with the packaged
  default. The file header carries a `TODO` to drop it once robostack ships
  a consistent pair.
- `ROS_DOMAIN_ID=0` in the manifest's activation env. The container runs on
  the default domain; a domain id inherited from the shell hides every
  container topic with no error.

With those three, a host peer connects to `localhost:7447` and sees the
container's graph. The image carries `rmw_zenoh_cpp` 0.6.6 and the host
0.6.9; the two interoperate. There is no `zenoh-router` task in `apps/aic`;
a second router on 7447 is a port conflict.

### 2. Layout

```
apps/aic/
├── README.md                     # application guide
├── docs/
│   ├── installation.md
│   └── demos/end-to-end-pipeline.md
├── pixi.toml / pixi.lock
├── colcon_defaults.yaml          # Kilted/py312; no -lpython linker flags
├── aic.repos                     # one entry: intrinsic-dev/aic @ SHA
├── compose.yaml                  # the `sim` service
├── compose.gpu.yaml              # NVIDIA overlay
├── .env.example                  # scene and GUI variables for compose; optional
├── config/zenoh/session.json5    # host session config, see decision 1
├── scripts/
│   ├── activate.sh
│   ├── prune_aic_source.sh       # COLCON_IGNORE everything but the allowlist
│   ├── check_interfaces.sh       # drift guard
│   ├── sim_smoke.sh              # joints, cameras and contract against a running simulator
│   └── reset_scene.sh            # runs inside the container
├── pai_aic/                      # ament_python
│   ├── package.xml  setup.py  setup.cfg  resource/pai_aic
│   ├── config/rosetta/aic.yaml   # the contract
│   ├── config/rosetta/policy_runner.yaml
│   ├── pai_aic/
│   │   ├── codecs.py             # twist <-> MotionUpdate
│   │   └── teleop.py             # terminal keyboard teleop
│   ├── launch/record.launch.py
│   ├── launch/deploy.launch.py
│   └── test/{test_codecs.py, test_contract.py, test_teleop.py}
└── external/ build/ install/ log/ datasets/ outputs/   ← gitignored
```

`common/scripts/install_ml_deps.sh` is promoted from
`apps/so_arm101/scripts/` because `apps/aic` is its second consumer. Both
manifests point at it. The script's contents do not change.

### 3. Pins and the drift guard

| What   | Pin                                                                                                          |
| ------ | ------------------------------------------------------------------------------------------------------------ |
| Image  | `ghcr.io/intrinsic-dev/aic/aic_eval@sha256:9aa2ffdbb946d38edde1bac7b5f02a44cfbea26e3b04a9c74e09f14c97472923` |
| Source | `https://github.com/intrinsic-dev/aic.git` @ `6546127c` (main, 2026-10-01)                                   |

The two move together. `aic.repos` carries a comment naming the digest it is
paired with, and `compose.yaml` carries a comment naming the SHA.

The host uses one AIC message package, `aic_control_interfaces`. The drift
guard compares, for every message and service in that package, the generated
type-description file (`share/aic_control_interfaces/{msg,srv}/*.json`, the
input to the ROS 2 type hash) between the host build and the image, and also
requires the two file sets to be identical. `ros2 interface show` is unfit
for this: it resolves an interface by suffix match, so `MotionUpdate` prints
`JointMotionUpdate`, and a comparison of its output would pass vacuously.

A mismatch means ROS 2 type hashes differ and the action topic silently
fails to connect, so this check runs on every PR.

### 4. Thin build

`install-source-deps` imports `common.repos` and `aic.repos` into
`external/`, then runs `scripts/prune_aic_source.sh`, which writes
`COLCON_IGNORE` into every package directory under `external/aic` except
the allowlist:

| Package                  | Why the host needs it              |
| ------------------------ | ---------------------------------- |
| `aic_control_interfaces` | `MotionUpdate`, the action message |

Its dependencies (`builtin_interfaces`, `geometry_msgs`, `std_msgs`,
`trajectory_msgs`) are robostack packages. An allowlist, so an upstream
package addition cannot pull the simulator into the host build.

`build` is `colcon build --base-paths . ../../common`, the same as
`apps/so_arm101`. It compiles Rosetta's packages, `aic_control_interfaces`
and `pai_aic`.

### 5. Simulator lifecycle

`compose.yaml` runs one service, `sim`, from the digest-pinned image with
host networking, the X11 socket mounted, `DISPLAY` passed through, and
`scripts/reset_scene.sh` bind-mounted read-only for decision 6. Its
`command` is the argument list for `aic_gz_bringup.launch.py`, which the
image entrypoint forwards: simulated time, no RViz, no engine, the task
board and a cable attached to the gripper, and the **scene** as
`${VAR:-default}` substitutions. Every default is the `trial_1` scene from
`aic_engine/config/sample_config.yaml`, so `sim-up` works with no `.env`.
`.env.example` lists the same variables with their defaults, including
`AIC_GAZEBO_GUI` and `AIC_CABLE_TYPE`; copying it to `.env` and editing is
how a user changes the scene.

`compose.gpu.yaml` adds `gpus: all`; machines without
`nvidia-container-toolkit` run the base file alone.

Docker Engine with the compose plugin is a host prerequisite the user
installs. Nothing Docker-related comes from conda.

Tasks: `sim-up` (`docker compose up -d`), `sim-down`, `sim-logs`, and
`sim-up-gpu` (the base file plus the overlay).

### 6. Scene reset

`reset-scene` runs `scripts/reset_scene.sh` inside the container through
`docker compose exec sim`; the script is bind-mounted read-only. It sources
`/ws_aic/install/setup.bash` and, in this order:

1. Removes the cable entity through gz transport
   (`/world/aic_world/remove`). The cable attached to the gripper would
   otherwise drag the arm off the home pose.
2. Homes the arm the way `aic_engine` does: deactivates `aic_controller`,
   calls `/scoring/reset_joints` with the six arm joints and the home
   positions from `aic_engine`'s sample config, and activates the
   controller again. The three calls run back to back because the arm is
   under effort control and falls while the controller is off.
3. Switches the controller's target mode to joint and back to Cartesian.
   The controller re-captures its Cartesian reference on that switch;
   without it the reference is the pose computed before the teleport and
   the controller pulls the arm away from home.
4. Tares the force/torque sensor and lets the arm come to rest.
5. Spawns a cable where `aic_engine` spawns it: the gripper frame's live
   position from TF plus the engine's gripper offset, with the engine's
   cable orientation, attached to the gripper, with the cable type from the
   environment. The cable plugin fixes the cable to the gripper at the
   relative pose it finds, so the spawn pose has to come from the live
   gripper pose; the launch file's default pose is only right while the arm
   is exactly at its initial pose.
6. Waits for the cable to load the gripper, then re-captures the reference
   again so the arm holds the loaded pose.

The arm settles a few degrees off the exact home joints under the cable's
load. That is the impedance controller's steady state and the same for
every episode.

Running inside the container keeps `aic_engine_interfaces`, the description
packages, `xacro` and TF off the host. The host build stays at one AIC
package.

### 7. Contract

`pai_aic/config/rosetta/aic.yaml`, schema v2:

```yaml
robot_type: aic
robot_interface: ros2

# Camera and adapter rate. Inside the controller's 10-30 Hz command band.
fps: 20

observations:
  observation.images.left:
    channel:
      topic: /left_camera/image
      type: sensor_msgs/msg/Image
      qos: { reliability: best_effort, history: keep_last, depth: 10 }
    align: { strategy: hold, timeline: header }
    apply: [resize: [480, 480]]
  observation.images.center: # same shape, /center_camera/image
  observation.images.right: # same shape, /right_camera/image

  observation.state:
    channel:
      topic: /joint_states
      type: sensor_msgs/msg/JointState
      qos: { reliability: reliable, history: keep_last, depth: 50 }
    align: { strategy: hold, timeline: header }
    # The six arm joints. Gripper finger joints are fixed during a task.
    select:
      [
        position.shoulder_pan_joint,
        position.shoulder_lift_joint,
        position.elbow_joint,
        position.wrist_1_joint,
        position.wrist_2_joint,
        position.wrist_3_joint,
      ]

actions:
  action:
    channel:
      topic: /aic_controller/pose_commands
      type: aic_control_interfaces/msg/MotionUpdate
      qos: { reliability: reliable, history: keep_last, depth: 10 }
      dtype: float64
      decoder: pai_aic.codecs:decode_motion_update
      encoder: pai_aic.codecs:encode_motion_update
      # A zero twist stops the arm. `hold` would keep it moving.
      safety: zeros
    # Arrival time on the recorder's clock: any command publisher aligns,
    # whatever clock stamps its header.
    align: { strategy: hold, timeline: receive }
    # Twist in the tool frame (gripper/tcp): m/s then rad/s.
    select: [linear_x, linear_y, linear_z, angular_x, angular_y, angular_z]
```

Choices locked here:

- **State** is the six arm joint positions through Rosetta's built-in
  `JointState` codec. No custom observation decoder exists. Radians stay
  radians: there is no LeRobot robot type to match.
- **Action** is the 6-dimensional **twist action** in the **tool frame**.
  `MotionUpdate` carries no gripper field, so there is no gripper dimension.
- Images resize to 480x480. The source is 1152x1024, so the aspect ratio
  changes, identically at training and deployment.
- The action aligns on the `receive` timeline. Commands come from whichever
  process drives the arm, and a publisher stamping headers with the wall
  clock while the recorder runs on simulated time would otherwise record a
  zero action stream.

This satisfies Rosetta 0.2's one-numeric-observation, one-action rule.

### 8. Codecs

`pai_aic/codecs.py`, referenced from the contract by path:

- `decode_motion_update(msg, spec) -> np.ndarray[6]`: reads
  `msg.velocity` linear then angular. Used by `rosetta_port` on recorded
  commands and by the recorder's action stream.
- `encode_motion_update(vec, spec, stamp_ns=None) -> MotionUpdate`: sets
  `header.frame_id = "gripper/tcp"`, `header.stamp` from `stamp_ns`,
  `velocity` from the vector, velocity mode, and the stiffness and damping
  diagonals AIC's reference policy uses (100, 100, 100, 50, 50, 50 and 40,
  40, 40, 15, 15, 15), so a deployed policy commands the impedance the
  demonstrations were recorded under. Zero feedforward wrench and zero
  wrench feedback gains.

`test/test_codecs.py` covers the round trip, the frame, the mode and the
impedance diagonals, and runs without ROS 2 middleware.

### 9. Teleop

`pai_aic/teleop.py` is a terminal keyboard teleop that reads its own
terminal in raw mode, the way Rosetta's `episode_keyboard_node` does. It
acts only when its terminal has focus, so Rosetta's recording keys and
ordinary typing in other terminals never move the arm.

- On start, asks `/aic_controller/change_target_mode` for Cartesian
  targets, the mode `MotionUpdate` commands need; a missing service is a
  warning, since the controller starts in that mode.
- Publishes `MotionUpdate` on `/aic_controller/pose_commands` at 20 Hz
  through `encode_motion_update`, so teleop and policy produce identical
  messages.
- Key layout follows AIC's Cartesian teleop: `a/d` X, `w/s` Y, `r/f` Z,
  `Shift+s/w` angular X, `Shift+a/d` angular Y, `q/e` angular Z, `k` slow
  (0.02 m/s, 0.02 rad/s), `l` fast (0.1 m/s, 0.1 rad/s), `Esc` quits.
  Frame is fixed to `gripper/tcp`; there is no frame toggle.
- A key press commands its axis for a hold window of 0.25 s. Terminal
  auto-repeat sustains it while the key is down; after the window the axis
  returns to zero. On exit the node publishes one zero twist.

### 10. Pixi manifest

| Field    | Value                                                                                                                                                                                                                                                                             |
| -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| channels | `robostack-kilted`, `conda-forge`                                                                                                                                                                                                                                                 |
| python   | `3.12.*`                                                                                                                                                                                                                                                                          |
| numpy    | `>=2.0,<2.3`                                                                                                                                                                                                                                                                      |
| platform | `linux-64`                                                                                                                                                                                                                                                                        |
| ROS      | `ros-kilted-ros-core`, `ros-kilted-rmw-zenoh-cpp`, `ros-kilted-rclpy`, `ros-kilted-launch-ros`, `ros-kilted-rosbag2`, `ros-kilted-rosbag2-storage-mcap`, `ros-kilted-cv-bridge`, `ros-kilted-geometry-msgs`, `ros-kilted-trajectory-msgs`, `ros-kilted-rosidl-default-generators` |
| build    | `colcon-common-extensions`, `cmake`, `ninja`, `c-compiler`, `cxx-compiler`, `vcstool`, `pre-commit`                                                                                                                                                                               |
| pypi     | `lerobot==0.6.0` with `dataset`, `async`, `training` extras; `huggingface-hub`; `scipy>=1.11,<2`                                                                                                                                                                                  |

Tasks, run from `apps/aic`:

| Task                  | Command                                                                                                                                                                                                                            |
| --------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `install-source-deps` | `vcs import external < ../../common/common.repos && vcs import external < aic.repos && bash scripts/prune_aic_source.sh`                                                                                                           |
| `install-ml-deps`     | `bash ../../common/scripts/install_ml_deps.sh`                                                                                                                                                                                     |
| `install-deps`        | depends on the two above                                                                                                                                                                                                           |
| `build`               | `colcon build --base-paths . ../../common` with `CMAKE_BUILD_TYPE=Release`                                                                                                                                                         |
| `clean`               | `rm -rf build install log`                                                                                                                                                                                                         |
| `lint`                | `pre-commit run --all-files`                                                                                                                                                                                                       |
| `check-interfaces`    | `bash scripts/check_interfaces.sh`                                                                                                                                                                                                 |
| `sim-up`              | `docker compose up -d`                                                                                                                                                                                                             |
| `sim-up-gpu`          | `docker compose -f compose.yaml -f compose.gpu.yaml up -d`                                                                                                                                                                         |
| `sim-down`            | `docker compose down`                                                                                                                                                                                                              |
| `sim-logs`            | `docker compose logs -f sim`                                                                                                                                                                                                       |
| `sim-smoke`           | `bash scripts/sim_smoke.sh`: the six arm joints and the three cameras publish, and the contract loads                                                                                                                              |
| `reset-scene`         | `docker compose exec sim bash /scripts/reset_scene.sh` (the script is bind-mounted read-only)                                                                                                                                      |
| `teleop`              | `ros2 run pai_aic teleop --ros-args -p use_sim_time:=true`                                                                                                                                                                         |
| `record`              | `ros2 launch pai_aic record.launch.py bag_base_dir:=datasets/aic/bags` (wraps `rosetta/episode_recorder_launch.py` with the contract and `use_sim_time:=true`)                                                                     |
| `record-keys`         | `ros2 run rosetta episode_keyboard_node`                                                                                                                                                                                           |
| `port-bags`           | `ros2 run rosetta rosetta_port --raw-dir datasets/aic/bags --contract $(ros2 pkg prefix pai_aic)/share/pai_aic/config/rosetta/aic.yaml --repo-id aic_cable_insertion --root datasets`                                              |
| `train`               | `lerobot-train --dataset.repo_id=aic_cable_insertion --dataset.root=datasets/aic_cable_insertion --policy.type=act --output_dir=outputs/train/aic_act --job_name=aic_act --policy.device=cuda --policy.push_to_hub=false`          |
| `deploy`              | `ros2 launch pai_aic deploy.launch.py pretrained_name_or_path:=outputs/train/aic_act/checkpoints/last/pretrained_model` (wraps `rosetta/policy_runner_launch.py` with the contract, `policy_runner.yaml` and `use_sim_time:=true`) |

Names carry no application prefix because the manifest is already scoped
to the application. `apps/so_arm101` still uses `so-arm-*` prefixes; that
cleanup is separate work.

### 11. The walkthrough the application guide documents

```
pixi run install-deps && pixi run build        # once
pixi run sim-up                                # terminal 1 (Gazebo window)
pixi run record                                # terminal 2
pixi run record-keys                           # terminal 3: r/s start/stop
pixi run teleop                                # terminal 4: drive the arm
pixi run reset-scene                           # between episodes
pixi run port-bags
pixi run train
pixi run deploy                                # with sim-up still running
ros2 action send_goal /run_policy rosetta_interfaces/action/RunPolicy "{prompt: 'insert the cable'}"
```

### 12. CI

| Workflow        | Trigger                                         | Runner       | Steps                                                                                                                                                                     |
| --------------- | ----------------------------------------------- | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `lint`          | always                                          | ubuntu-22.04 | unchanged                                                                                                                                                                 |
| `aic`           | PR and push touching `apps/aic/**`, `common/**` | ubuntu-24.04 | `install-source-deps` → `build` → `check-interfaces` → `pytest pai_aic/test`                                                                                              |
| `aic-sim-smoke` | nightly and `workflow_dispatch`                 | ubuntu-24.04 | `sim-up` headless → wait for `/joint_states` to list the six arm joints and for one message on each camera topic → load the contract with `rosetta`'s loader → `sim-down` |

PR CI never starts the simulator: camera rendering on a GPU-less runner is
llvmpipe, slow and flaky. The smoke job needs no training and no GPU.

### 13. Documentation

- `apps/aic/README.md`: what the scenario is, prerequisites (Docker, X11,
  optional NVIDIA toolkit), quick start, the task table, troubleshooting
  (empty topics → run `check-interfaces`; the Gazebo window and Wayland;
  camera rate without a GPU; the zenoh session config).
- `apps/aic/docs/installation.md` and `apps/aic/docs/demos/end-to-end-pipeline.md`
  mirror the SO-ARM101 pages.
- Root `README.md` gains the catalog row: AIC cable insertion, `apps/aic/`,
  Kilted, Gazebo (container), Active.
- `2026-06-30-pai-aic-integration-design.md` and
  `../plans/2026-09-11-aic-app-docker-first.md` get a status line pointing
  here.

## Acceptance

From a fresh clone on a `linux-64` machine with Docker, the documented
commands bring the simulator up, record episodes, port them, train an ACT
policy and deploy it, and the arm moves under policy control in the AIC
scene. Cable insertion success is a data and training question outside this
integration.

## Verification items for the implementation plan

Assumptions this design makes that the plan must check before building on
them:

1. Rosetta 0.2.0 and `lerobot_rosetta` build and import on Kilted with
   Python 3.12 and LeRobot 0.6.0.
2. `docker compose exec sim` can call `/scoring/reset_joints`, delete the
   cable entity and respawn it with `spawn_cable.launch.py` while the
   simulator keeps running, and the respawned cable is attached to the
   gripper. The cable entity name must be confirmed from the launch file.
3. `aic_gz_bringup.launch.py` accepts the scene arguments from the compose
   command and spawns the `trial_1` scene.
4. A host `rmw_zenoh_cpp` peer with the settings of decision 1 sees the
   container's topics with ACL off. Checked: it does, and without the
   session config and the domain pin it does not.
5. The contract loads with `rosetta`'s `load_contract` against the thin
   build, which proves the codec paths, the `JointState` select names and
   the `header` timeline on `MotionUpdate`.
6. `prune_aic_source.sh` leaves exactly one AIC package visible to colcon.
7. Terminal auto-repeat delivers key events fast enough for the 0.25 s hold
   window in the teleop; tune the window if the arm stutters.

## Alternatives considered

- **Second container for our own code**, host without ROS. Rejected: every
  edit to `pai_aic` or the contract would rebuild an image, and the SO-ARM101
  application already runs the same pipeline natively.
- **Lyrical on the host.** Rejected: cross-version zenoh interoperability is
  an avoidable risk, and Rosetta is pure Python so Kilted costs nothing.
- **AIC-native deploy path** (`aic_model` + a LeRobot policy plugin).
  Deferred: it only matters for `aic_engine` scoring, which is out of scope,
  and the old implementation targets LeRobot 0.5 internals.
- **26-dimensional fused state from `/observations`.** Rejected: it needs
  a custom decoder on a header-less message, and it folds in `tcp_error`,
  the controller's own target error, which leaks the command into the
  observation. Tool pose is a deterministic function of the joints.
- **AIC's `aic_teleoperation` on the host.** Rejected: `pynput` captures
  keystrokes desktop-wide, so Rosetta's recording keys and any typing move
  the arm.
- **LeRobot's `aic_controller` robot type** (`lerobot_robot_aic`). Rejected:
  pinned to LeRobot 0.5.1 through a fork of `lerobot-ros`.
- **Host-side scene spawning** (`aic_spawn_scene.py`). Rejected: the launch
  file already takes every scene argument, and spawning on the host drags
  `aic_description` and `xacro` into the thin build.
- **`base_link` as the action frame.** Rejected: the demonstrations and the
  policy must agree on one frame, and the tool frame makes the learned
  motion relative to the gripper.

## Out of scope

- `aic_engine` scoring and the `InsertCable` action.
- AIC's MuJoCo and Isaac Lab ports.
- Gripper control. `MotionUpdate` has no gripper field and AIC spawns no
  gripper controller.
- Real UR5e hardware.
- Task-name prefix cleanup in `apps/so_arm101`.
- Fixing `install_ml_deps.sh`'s torch wheel choice for standard NVIDIA.

## Risks

1. **Silent interface drift** between image and source. Mitigated by digest
   and SHA pins and the drift guard on every PR.
2. **Image tag movement.** `latest` has moved before. The compose file uses
   the digest, so builds are reproducible; a bump is a deliberate edit of
   both pins.
3. **Teleop ergonomics.** A terminal-focused teleop with a hold window may
   feel less fluid than a global listener. The window is a tunable.
4. **Cable respawn.** If step 2 of the verification list fails, the
   fallback is `sim-down` + `sim-up` between episodes, slower but correct.
5. **Rendering on headless runners.** The nightly smoke job may be slow
   under llvmpipe; it asserts message arrival, not image content.
