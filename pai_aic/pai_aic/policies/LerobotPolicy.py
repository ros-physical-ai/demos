# Copyright 2026 demos contributors
# Apache 2.0

"""
LerobotPolicy: a generic aic_model.Policy that loads any LeRobot-trained
checkpoint and publishes cartesian twist commands to aic_controller.

Mirrors the structure of aic_example_policies/RunACT.py but parameterizes
the policy type so it can be swapped via a launch arg.

Usage:
    ros2 run aic_model aic_model --ros-args \
        -p use_sim_time:=true \
        -p policy:=pai_aic.policies.LerobotPolicy \
        -p checkpoint_path:=outputs/train/aic_act/checkpoints/last/pretrained_model \
        -p policy_type:=act
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from aic_model.policy import Policy
from aic_task_interfaces.msg import Task

from pai_aic.converters import (
    decode_aic_observation,
    encode_aic_motion_update,
)


class LerobotPolicy(Policy):
    """Generic LeRobot policy loaded into aic_model."""

    def __init__(self, parent_node):
        super().__init__(parent_node)
        node = parent_node

        # 1. Resolve parameters
        checkpoint = (
            node.declare_parameter("checkpoint_path", "")
            .get_parameter_value()
            .string_value
        )
        if not checkpoint:
            raise RuntimeError("checkpoint_path parameter is required")

        self.checkpoint = Path(checkpoint)
        self.policy_type = (
            node.declare_parameter("policy_type", "act")
            .get_parameter_value()
            .string_value
        )
        self.image_scale = float(
            node.declare_parameter("image_scale", "0.25")
            .get_parameter_value()
            .string_value
        )
        self.loop_rate_hz = float(
            node.declare_parameter("loop_rate_hz", "20")
            .get_parameter_value()
            .string_value
        )

        # 2. Load policy + normalization stats
        self.device = torch.device(
            "cuda" if torch.cuda.is_available() else "cpu"
        )
        self.policy, self.stats = self._load_policy_and_stats(
            self.policy_type, self.checkpoint, self.device
        )

        # 3. Feature key mapping (must match the contract's `key:` field)
        self.feature_keys = {
            "left":   "observation.images.left_camera",
            "center": "observation.images.center_camera",
            "right":  "observation.images.right_camera",
        }

        self.get_logger().info(
            f"LerobotPolicy ready: type={self.policy_type} "
            f"checkpoint={self.checkpoint} device={self.device} "
            f"loop_rate_hz={self.loop_rate_hz}"
        )

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------
    def _load_policy_and_stats(self, policy_type, checkpoint, device):
        """Load a LeRobot policy and its normalization stats."""
        if policy_type != "act":
            raise NotImplementedError(
                f"Policy type '{policy_type}' not yet wired. "
                f"Extend _load_policy_and_stats() to support it."
            )

        # Lazy imports so a missing LeRobot dep doesn't break aic_model startup
        # for users on other policy paths.
        from lerobot.policies.act.modeling_act import ACTPolicy
        from lerobot.policies.act.configuration_act import ACTConfig
        import draccus
        from safetensors.torch import load_file

        with open(checkpoint / "config.json") as f:
            cfg = json.load(f)
        cfg.pop("type", None)
        config = draccus.decode(ACTConfig, cfg)

        policy = ACTPolicy(config).to(device).eval()
        policy.load_state_dict(load_file(checkpoint / "model.safetensors"))

        stats_path = (
            checkpoint
            / "policy_preprocessor_step_3_normalizer_processor.safetensors"
        )
        # Move stats onto the same device as the policy/observations so that
        # normalization (state - mean) / std doesn't mix cuda and cpu tensors.
        stats = {k: v.to(device) for k, v in load_file(stats_path).items()}

        return policy, stats

    # ------------------------------------------------------------------
    # Inference helpers
    # ------------------------------------------------------------------
    def _img_to_tensor(self, raw_img, mean, std):
        """sensor_msgs/Image → normalized CHW float tensor."""
        img = np.frombuffer(raw_img.data, dtype=np.uint8).reshape(
            raw_img.height, raw_img.width, 3
        )
        if self.image_scale != 1.0:
            img = cv2.resize(
                img, None,
                fx=self.image_scale, fy=self.image_scale,
                interpolation=cv2.INTER_AREA,
            )
        t = (
            torch.from_numpy(img)
            .permute(2, 0, 1)
            .float()
            .div(255.0)
            .unsqueeze(0)
            .to(self.device)
        )
        return (t - mean) / std

    def _prepare_observations(self, obs_msg):
        """Observation.msg → dict of normalized tensors for LeRobot policy."""
        # State (26-dim, raw SI units)
        state_np = decode_aic_observation(obs_msg, spec=None)
        state = (
            torch.from_numpy(state_np).float().unsqueeze(0).to(self.device)
        )
        m = self.stats["observation.state.mean"].view(1, -1)
        s = self.stats["observation.state.std"].view(1, -1)
        out = {"observation.state": (state - m) / s}

        # Cameras (3x from Observation.msg)
        cam_map = {
            "left":   obs_msg.left_image,
            "center": obs_msg.center_image,
            "right":  obs_msg.right_image,
        }
        for cam_name, img_msg in cam_map.items():
            mean = (
                self.stats[f"observation.images.{cam_name}_camera.mean"]
                .view(1, 3, 1, 1)
            )
            std = (
                self.stats[f"observation.images.{cam_name}_camera.std"]
                .view(1, 3, 1, 1)
            )
            out[self.feature_keys[cam_name]] = self._img_to_tensor(
                img_msg, mean, std
            )

        return out

    # ------------------------------------------------------------------
    # Main policy loop
    # ------------------------------------------------------------------
    def insert_cable(
        self,
        task: Task,
        get_observation,
        move_robot,
        send_feedback,
    ) -> bool:
        self.get_logger().info(
            f"LerobotPolicy.insert_cable() — task='{task}'"
        )
        try:
            self.policy.reset()
        except Exception:
            pass  # not all policies implement reset()

        period = 1.0 / self.loop_rate_hz
        send_feedback("policy running")

        # Bound the loop by the task's time limit so the worker thread always
        # terminates (returns) instead of running forever. aic_model starts
        # insert_cable() in a plain thread it cannot kill; a `while True` loop
        # would keep commanding the robot after a trial is canceled/timed out
        # and leak into the next trial. time_limit is in seconds.
        deadline = time.time() + float(task.time_limit)

        while time.time() < deadline:
            t0 = time.time()
            obs = get_observation()
            if obs is None:
                self.sleep_for(0.01)
                continue

            obs_tensors = self._prepare_observations(obs)

            with torch.inference_mode():
                norm_action = self.policy.select_action(obs_tensors)

            action = (
                norm_action
                * self.stats["action.std"].view(1, -1)
                + self.stats["action.mean"].view(1, -1)
            )[0].cpu().numpy()

            stamp_ns = (
                obs.center_image.header.stamp.sec * 1_000_000_000
                + obs.center_image.header.stamp.nanosec
            )
            motion_update = encode_aic_motion_update(
                action, spec=None, stamp_ns=int(stamp_ns)
            )
            move_robot(motion_update=motion_update)
            send_feedback("step")

            elapsed = time.time() - t0
            self.sleep_for(max(0.0, period - elapsed))

        self.get_logger().info("LerobotPolicy.insert_cable() — time limit reached")
        return True
