#!/usr/bin/env python3
"""Convert a LeRobot PyTorch policy to ONNX with chunked inference and
dataset_stats normalization baked into the graph.

Example:
    pixi run onnx-convert \\
        --policy francocipollone/rospai_act_sim_arm101_place_cubes_on_tray \\
        --out   ./policy.onnx

The output graph has:
  inputs:  observation.images.wrist (1,3,H,W) float32,
           observation.images.static (1,3,H,W) float32,
           observation.state          (1,state_dim) float32
  output:  action                    (1,chunk_size,action_dim) float32

`dataset_stats.json` is loaded from the policy artifact and the
normalization/denormalization is baked in as `Sub` + `Div` + `Mul` + `Add`
initializers on each input/output tensor — so the C++ node does NO stats work.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import torch


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--policy", required=True,
                   help="HF repo ID or local path containing the trained policy.")
    p.add_argument("--out", required=True, help="Output .onnx path.")
    p.add_argument("--policy-type", default="act",
                   choices=["act", "diffusion", "pi0", "pi05", "smolvla"])
    p.add_argument("--chunk-size", type=int, default=None,
                   help="Override the model's chunk size (default: read from config).")
    p.add_argument("--no-bake-stats", action="store_true",
                   help="Do not bake dataset_stats into the graph (escape hatch).")
    return p.parse_args()


def make_policy(args):
    if args.policy_type == "act":
        from lerobot.policies.act.modeling_act import ACTPolicy as PolicyCls
    else:
        from lerobot.policies.factory import get_policy_class
        PolicyCls = get_policy_class(args.policy_type)
    return PolicyCls.from_pretrained(args.policy, local_files_only=False).to("cpu").eval()


def build_dummy(input_features):
    out = {}
    for name, feat in input_features.items():
        if feat.type == "VISUAL":
            c, h, w = feat.shape
            out[name] = torch.randn(1, c, h, w, dtype=torch.float32)
        elif feat.type == "STATE":
            out[name] = torch.randn(1, feat.shape[0], dtype=torch.float32)
        else:
            raise ValueError(f"unsupported feature type {feat.type} for {name}")
    return out


def load_stats(args):
    if args.no_bake_stats:
        return None
    stats_path = Path(args.policy) / "dataset_stats.json"
    if not stats_path.exists():
        print(f"[WARN] {stats_path} not found; running without baked stats.",
              file=sys.stderr)
        return None
    return json.loads(stats_path.read_text())


def main() -> int:
    args = parse_args()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    policy_cpu = make_policy(args)
    feats = policy_cpu.config.input_features
    dummy = build_dummy(feats)
    input_names = list(dummy.keys())
    stats = load_stats(args)

    chunk_size = args.chunk_size or int(policy_cpu.config.chunk_size)
    action_dim = int(policy_cpu.config.output_features["action"].shape[0])

    # Flat module: forward takes positional inputs in `input_names` order.
    class Wrapped(torch.nn.Module):
        def __init__(self, base, stats, keys, chunk_size, action_dim):
            super().__init__()
            self.base = base
            self.stats = stats
            self.keys = list(keys)
            self.chunk_size = chunk_size
            self.action_dim = action_dim

        def forward(self, *inputs):
            batch = dict(zip(self.keys, inputs))

            if self.stats is not None:
                normed = {}
                for k, v in batch.items():
                    s = (self.stats.get("observation.features", {})
                                       .get(k) or self.stats.get(k))
                    if s is None:
                        normed[k] = v
                        continue
                    mean = torch.tensor(s["mean"], dtype=v.dtype).reshape(1, -1)
                    std  = torch.tensor(s["std"],  dtype=v.dtype).reshape(1, -1)
                    if v.dim() == 4:  # image: broadcast per-channel.
                        mean = mean.reshape(1, -1, 1, 1)
                        std  = std.reshape(1, -1, 1, 1)
                    normed[k] = (v - mean) / std
            else:
                normed = batch

            out = self.base(normed)
            action = out[0] if isinstance(out, tuple) else out
            # Force (B, chunk_size, action_dim).
            if action.dim() == 2:
                action = action.unsqueeze(1).expand(-1, self.chunk_size, -1)
            elif action.dim() == 3 and action.shape[1] == 1:
                action = action.expand(-1, self.chunk_size, -1)
            elif action.dim() == 3 and action.shape[1] != self.chunk_size:
                # Interpolate in chunk dim by repetition — only valid for chunk_size=1 input.
                if action.shape[1] == 1:
                    action = action.expand(-1, self.chunk_size, -1)
                else:
                    raise ValueError(
                        f"model chunk {action.shape[1]} != requested {self.chunk_size}; "
                        "use --chunk-size or --no-bake-stats")

            if self.stats is not None:
                a_stats = self.stats.get("action")
                if a_stats is not None:
                    mean = torch.tensor(a_stats["mean"], dtype=action.dtype).reshape(1, 1, -1)
                    std  = torch.tensor(a_stats["std"],  dtype=action.dtype).reshape(1, 1, -1)
                    action = action * std + mean
            return action

    full = Wrapped(policy_cpu, stats, input_names, chunk_size, action_dim)
    full.eval()

    dummy_tensors = tuple(dummy[k] for k in input_names)
    torch.onnx.export(
        full,
        dummy_tensors,
        str(out_path),
        export_params=True,
        opset_version=18,
        do_constant_folding=True,
        input_names=input_names,
        output_names=["action"],
        dynamic_axes={n: {0: "batch"} for n in input_names} | {"action": {0: "batch"}},
        dynamo=False,
    )
    print(f"OK — wrote {out_path} ({out_path.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
