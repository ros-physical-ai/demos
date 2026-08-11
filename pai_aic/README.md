# `pai_aic` — AIC cable-insertion scenario for demos' pipeline

This package bridges demos' Record → Train → Deploy pipeline to the
[AI for Industry Challenge](https://github.com/intrinsic-dev/aic) sim
(UR5 + task board + impedance controller + cable-insertion task).

The goal is to **reuse demos' tooling** (Rosetta contracts, `episode_recorder`,
`port_bags`, `lerobot-train`) and produce trained policies that deploy back
into the AIC sim via **two parallel paths**:

- **`aic-deploy-aic-model`** — `aic_model` lifecycle node + `LerobotPolicy`
  class. AIC-native, `aic_engine`-compatible.
- **`aic-deploy-rosetta`** — `rosetta_client_node` from demos. Demos-native,
  async inference.

Both paths consume the same contract (`pai_data_collection/config/rosetta/aic.yaml`)
and the same LeRobot checkpoint. Run both and compare control smoothness,
latency, and behavior.

> [!TIP]
> This README is a quick reference. For the full step-by-step walkthrough
> (setup → launch → spawn scene → record → convert → train → deploy) see
> [`docs/demos/aic-scenario.md`](../docs/demos/aic-scenario.md).

## Setup

The AIC scenario runs in its own `aic` Pixi environment (from-source Gazebo)
with its own `install_aic/` build tree, kept separate from the base SO-ARM101
demo so neither has to build the other's dependencies:

```bash
pixi install
pixi run setup-aic   # import pai.repos + aic.repos (aic packages + UR5 + gz-from-source + ros-controls)
pixi run aic-build   # build into install_aic/ (long: compiles the gz stack from source)
```

`aic.repos` provides both the `aic_*` packages (`aic_bringup`, `aic_engine`,
`aic_model`, `aic_adapter`, `aic_controller`, …) and their build dependencies
(UR5 + from-source Gazebo + ros-controls). The base demo's `pixi run setup`
imports only `pai.repos`, so it never pulls in — or builds — any of this.

All `aic-*` tasks below run in the `aic` environment automatically; for other
commands use `pixi run -e aic …` or `pixi shell -e aic`.

## Recording workflow

```bash
# Terminal 1 — Zenoh
pixi run start_zenoh

# Terminal 2 — AIC sim + adapter (UR5 + impedance controller, empty scene)
pixi run aic-gz

# Terminal 3 — Spawn the scene (task board + per-rail cards/ports + cable)
pixi run aic-spawn-scene   # re-run with `-- --clear` to reset between episodes

# Terminal 4 — Recorder
pixi run aic-record

# Terminal 5 — Keyboard controller (demos' episode_keyboard_node)
pixi run ros2 run rosetta episode_keyboard_node

# Terminal 6 — Teleop (LeRobot's aic_keyboard_ee, or scripted topic pub)
pixi run lerobot-teleoperate \
    --robot.type=aic_controller --robot.id=aic \
    --teleop.type=aic_keyboard_ee --teleop.id=aic \
    --robot.teleop_target_mode=cartesian --robot.teleop_frame_id=base_link \
    --display_data=false
```

> [!NOTE]
> Do **not** use `aic-engine` for teleoperated recording. Its trial loop is
> built to evaluate an autonomous policy (spawn → wait for model → score →
> reset), not for a human collecting demonstrations. Spawn the scene with
> `aic-spawn-scene` and manage resets yourself. `aic-engine` is used at
> evaluation time — see the [full guide](../docs/demos/aic-scenario.md#evaluating-with-aic_engine).

## Training workflow

```bash
pixi run aic-train
```

## Deploy workflow

```bash
# Path A — aic_model + LerobotPolicy (AIC-native)
pixi run aic-deploy-aic-model

# Path B — rosetta_client_node (demos-native)
pixi run aic-deploy-rosetta
```

## Layout

The 26-dim state vector and 6-dim action vector are locked in
`pai_aic/converters.py`. See the docstrings there for the exact axis
ordering.

| Dim | Range | Meaning |
|---|---|---|
| 0-6 | tcp_pose | x, y, z (3) + qx, qy, qz, qw (4) |
| 7-9 | tcp_linear_velocity | x, y, z |
| 10-12 | tcp_angular_velocity | x, y, z |
| 13-18 | tcp_error | x, y, z, rx, ry, rz |
| 19-25 | joint_positions | 6 arm joints + 1 gripper |

| Action dim | Meaning |
|---|---|
| 0-2 | linear twist (m/s) |
| 3-5 | angular twist (rad/s) |

## See also

- Full walkthrough: [`docs/demos/aic-scenario.md`](../docs/demos/aic-scenario.md)
- AIC upstream: https://github.com/intrinsic-dev/aic
