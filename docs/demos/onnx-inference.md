# C++ ONNX inference (`pai_onnx_inference`)

A drop-in replacement for the Python `rosetta.policy_bridge_node.PolicyBridge`,
written in C++ and backed by ONNX Runtime. Same contract YAML, same `/run_policy`
action server, same action chunking, same dataset-stats normalization — with
no PyTorch at deploy time.

## Why

- Smaller deploy footprint (no PyTorch / cuDNN in the C++ image).
- Faster cold start (~100 ms vs ~3-8 s for the Python node).
- Provider portability (CPU today, optional CUDA EP).

## Convert a LeRobot policy to ONNX

```bash
pixi run onnx-convert --policy <hf_repo_or_local_path> --out ./policy.onnx
```

The script (`scripts/torch_to_onnx.py`) exports the policy's chunked forward
pass and bakes `dataset_stats.json` into the graph, so the C++ node needs no
stats files at runtime.

## Launch

```bash
# Terminal 1
pixi run start_zenoh

# Terminal 2 — Gazebo (or MuJoCo / real hardware via `so-arm-mujoco` / `so-arm-real`)
pixi run so-arm-gz

# Terminal 3 — the C++ inference node
ros2 launch pai_onnx_inference inference.launch.py \
    model_path:=/abs/path/to/policy.onnx \
    contract_path:=$(ros2 pkg prefix pai_data_collection)/share/pai_data_collection/config/rosetta/so_arm101.yaml \
    use_sim_time:=true

# Terminal 4 — trigger inference
ros2 action send_goal /run_policy \
    rosetta_interfaces/action/RunPolicy "{prompt: 'place cubes on tray'}"
```

## Parameters

| Param | Default | Notes |
|---|---|---|
| `model_path` | — | required, abs path to `.onnx` |
| `contract_path` | — | required, abs path to YAML |
| `ep_requested` | `auto` | `auto`, `cpu`, `cuda` |
| `ep_policy` | `fallback` | `fallback`, `strict` |
| `observation_timeout_s` | `1.0` | drop ticks if obs are older |
| `drop_warn_threshold` | `5` | consecutive dropped ticks before WARN |
| `onnx_error_policy` | `log_and_drop` | `log_and_drop`, `publish_hold`, `fatal` |
| `sim_time_watchdog_s` | `2.0` | pause publish if sim time stalls |
| `use_sim_time` | `false` | set `true` in sim |

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `Ort::Session load failed` | bad `.onnx` (opset, missing initializers) | re-run `pixi run onnx-convert` |
| schema mismatch warning | ONNX I/O keys differ from contract | confirm export with the matching `--policy-type` |
| `CUDA EP not available` | `libonnxruntime-cuda` not installed | install `onnxruntime-cuda` (CPU fallback is on by default) |
| no `/forward_position_controller/commands` | obs never arrived | check `ros2 topic hz /joint_states` and the two cameras |
| robot stalls | sim time frozen | check `/clock`; bump `sim_time_watchdog_s` |

## Architecture

The package is split into five testable units (`ContractParser`, `AlignedBuffer`,
`ChunkQueue`, `OnnxRunner`, `ObservationAssembler`) plus the `InferenceNode`
that wires them. ONNX runs on a dedicated thread so CUDA sync never blocks the
ROS executor. See [`docs/superpowers/specs/2026-06-25-pai-onnx-inference-design.md`](../superpowers/specs/2026-06-25-pai-onnx-inference-design.md)
for the design spec.