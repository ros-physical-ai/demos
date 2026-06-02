# Copyright 2026 Franco Cipollone
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""SO-ARM101 ROS 2 -> Rerun visualizer.

Spatial layer (standard ROS 2):
  * robot mesh posed via pinocchio forward kinematics from ``/joint_states``,
  * camera images logged as ``rr.Image`` with ``rr.Pinhole`` projection,
  * joint-position time series.

ML layer (tensors):
  * predicted action-chunk trajectory (``trajectory_msgs/JointTrajectory``)
    turned into a 3D end-effector path via forward kinematics and drawn as a
    fading ``rr.LineStrips3D`` + ``rr.Points3D`` extending from the gripper.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from functools import partial
from typing import Optional

import numpy as np
import rclpy
import rerun as rr
import rerun.blueprint as rrb
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from pai_rerun_visualizer.robot_model import RobotModel

_VIEW_COORDS = {
    "RDF": rr.ViewCoordinates.RDF,
    "FLU": rr.ViewCoordinates.FLU,
    "RUB": rr.ViewCoordinates.RUB,
    "FRD": rr.ViewCoordinates.FRD,
}

_MONO_ENCODINGS = {"mono8", "mono16", "32fc1", "16uc1"}
_cv_bridge = CvBridge()


def stamp_to_datetime64(stamp) -> np.datetime64:
    """Convert a ROS 2 ``builtin_interfaces/Time`` stamp to ``np.datetime64`` nanoseconds."""
    return np.datetime64(Time.from_msg(stamp).nanoseconds, "ns")


def image_to_numpy(msg: Image) -> tuple[Optional[np.ndarray], Optional[str]]:
    """Convert a ``sensor_msgs/Image`` to a numpy array + Rerun color model.

    Uses ``cv_bridge`` for robust handling of all ROS image encodings (bayer,
    yuv, 16-bit, float, etc.).  Mono/depth images are kept as single-channel
    ``uint8``; all colour images are converted to ``rgb8``.
    """
    is_mono = (msg.encoding or "").lower() in _MONO_ENCODINGS
    desired = "mono8" if is_mono else "rgb8"
    try:
        img = _cv_bridge.imgmsg_to_cv2(msg, desired_encoding=desired)
    except Exception:
        return None, None
    return img, "L" if is_mono else "RGB"


@dataclass
class CameraCfg:
    """Configuration for a single camera, used to build subscriptions and log to Rerun."""

    name: str
    image_topic: str
    info_topic: str


@dataclass
class Topics:
    """Grouped ROS topic names for all subscriptions in the visualizer node."""

    robot_description: str
    joint_states: str
    forward_commands: Optional[str]
    action_chunk: str
    cameras: list[CameraCfg] = field(default_factory=list)


class VisualizerNode(Node):
    """ROS 2 node that subscribes to robot state and camera topics and logs them to Rerun."""

    def __init__(
        self,
        topics: Topics,
        cmd_joint_order: list[str],
        ee_frame: str,
        camera_xyz: str,
        trajectory_color: tuple[int, int, int],
        viz_max_hz: float = 30.0,
    ) -> None:
        """Initialise the node, build subscriptions, and configure Rerun entities.

        Args:
            topics: Grouped ROS topic names for all subscriptions.
            cmd_joint_order: Ordered joint names matching ``forward_commands`` array indices.
            ee_frame: URDF frame used as the end-effector for trajectory FK.
            camera_xyz: Rerun ``ViewCoordinates`` key (e.g. ``"RDF"``) for camera orientation.
            trajectory_color: RGB tuple for the predicted action-chunk path overlay.
            viz_max_hz: Maximum rate (Hz) at which 3D transforms and camera images are
                sent to Rerun.  Scalar time-series are exempt and always logged at the
                full joint-state rate.  Raising this increases visual smoothness but
                also gRPC channel pressure.

        """
        super().__init__("pai_rerun_visualizer")
        self._topics = topics
        self._cmd_joint_order = list(cmd_joint_order)
        self._ee_frame = ee_frame
        self._camera_xyz = _VIEW_COORDS.get(camera_xyz.upper(), rr.ViewCoordinates.RDF)
        self._traj_color = trajectory_color
        self._viz_min_interval_s: float = 1.0 / max(viz_max_hz, 0.1)

        # Two independent pinocchio models avoid cross-thread state clobbering:
        # one tracks the live robot pose, the other does trajectory FK.
        self._model: Optional[RobotModel] = None
        self._traj_model: Optional[RobotModel] = None
        self._model_lock = threading.Lock()
        self._logged_meshes = False
        self._cam_info: dict[str, tuple[np.ndarray, int, int, str]] = {}
        self._last_robot_viz_t: float = 0.0  # wall time of last 3D transform log
        self._last_image_viz_t: dict[str, float] = {}  # per-camera wall time of last image log

        cg = ReentrantCallbackGroup()

        urdf_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, topics.robot_description, self._on_robot_description, urdf_qos, callback_group=cg
        )
        self.create_subscription(
            JointState, topics.joint_states, self._on_joint_states, qos_profile_sensor_data, callback_group=cg
        )

        for cam in topics.cameras:
            self.create_subscription(
                Image,
                cam.image_topic,
                partial(self._on_image, cam_name=cam.name),
                qos_profile_sensor_data,
                callback_group=cg,
            )
            self.create_subscription(
                CameraInfo,
                cam.info_topic,
                partial(self._on_camera_info, cam_name=cam.name),
                qos_profile_sensor_data,
                callback_group=cg,
            )

        if topics.forward_commands:
            self.create_subscription(
                Float64MultiArray,
                topics.forward_commands,
                self._on_forward_commands,
                QoSProfile(depth=10),
                callback_group=cg,
            )

        self.create_subscription(
            JointTrajectory, topics.action_chunk, self._on_action_chunk, QoSProfile(depth=2), callback_group=cg
        )

        self.get_logger().info("pai_rerun_visualizer started; waiting for robot_description...")

    # -- URDF ----------------------------------------------------------
    def _on_robot_description(self, msg: String) -> None:
        """Parse the URDF from the robot_description topic and (re-)build both kinematic models."""
        try:
            model = RobotModel(msg.data)
            traj_model = RobotModel(msg.data)
        except Exception as exc:
            self.get_logger().error(f"Failed to build kinematic model from URDF: {exc}")
            return
        with self._model_lock:
            self._model = model
            self._traj_model = traj_model
            self._logged_meshes = False
        self.get_logger().info(
            f"Built kinematic model: {len(model.visuals)} visual meshes, ee_frame='{self._ee_frame}'"
            + ("" if model.has_frame(self._ee_frame) else " (NOT found in URDF!)")
        )

    def _log_static_meshes(self, visuals: list) -> None:
        """Log visual mesh assets and their static link-to-visual offsets to Rerun.

        Called with a snapshot of the model's visual list captured outside the
        model lock.  Each mesh is logged as a static ``rr.Asset3D`` under
        ``robot/<link>/visuals/visual_<i>``, with an optional static
        ``Transform3D`` for any non-identity ``<origin>`` or non-unit scale.

        Hierarchy::

            robot/<link>                          # dynamic Transform3D (updated by FK)
            └── robot/<link>/visuals/visual_<i>  # static Asset3D + optional offset
        """
        for i, vis in enumerate(visuals):
            albedo = (vis.color * 255).astype(np.uint8) if vis.color is not None else None
            rr.log(
                f"robot/{vis.link}/visuals/visual_{i}",
                rr.Asset3D(path=vis.mesh_path, albedo_factor=albedo),
                static=True,
            )
            has_offset = not np.allclose(vis.origin, np.eye(4))
            has_scale = not np.allclose(vis.scale, 1.0)
            if has_offset or has_scale:
                rr.log(
                    f"robot/{vis.link}/visuals/visual_{i}",
                    rr.Transform3D(
                        translation=vis.origin[:3, 3],
                        mat3x3=vis.origin[:3, :3],
                        scale=vis.scale,
                    ),
                    static=True,
                )

    # -- joint states / robot pose -------------------------------------
    def _on_joint_states(self, msg: JointState) -> None:
        """Update joint time series, run FK, and refresh robot pose and camera transforms.

        The model lock is held **only** for FK computation (fast numpy/pinocchio ops).
        All Rerun I/O runs after the lock is released so that gRPC back-pressure
        cannot block other callbacks from acquiring the lock.  3D transforms are
        rate-gated to ``viz_max_hz``; scalar time-series are always logged.
        """
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        rr.set_time("ros_time", timestamp=np.datetime64(stamp_ns, "ns"))

        name_to_pos = {name: float(msg.position[i]) for i, name in enumerate(msg.name) if i < len(msg.position)}
        for name, value in name_to_pos.items():
            rr.log(f"state/position/{name}", rr.Scalars(value))

        # ── FK (fast: pinocchio + numpy) ──────────────────────────────
        # Capture all results inside the lock; copy arrays so Rerun I/O
        # can run safely after the lock is released.
        pending_visuals: list = []
        link_poses: dict[str, np.ndarray] = {}
        cam_snapshots: list[tuple] = []

        with self._model_lock:
            model = self._model
            if model is None:
                return
            if not self._logged_meshes:
                pending_visuals = list(model.visuals)
                self._logged_meshes = True
            model.set_joint_positions(name_to_pos)
            for vis in model.visuals:
                if vis.link not in link_poses:
                    pose = model.frame_pose(vis.link)
                    if pose is not None:
                        link_poses[vis.link] = pose.copy()
            for cam in self._topics.cameras:
                info = self._cam_info.get(cam.name)
                if info is None:
                    continue
                frame_id = info[3]
                pose = model.frame_pose(frame_id)
                if pose is not None:
                    cam_snapshots.append((cam.name, pose.copy()))

        # ── Rerun I/O (slow: serialise + gRPC) ───────────────────────
        if pending_visuals:
            self._log_static_meshes(pending_visuals)
        # Rate-gate 3D transforms to _VIZ_MAX_HZ.  Scalar plots are exempt
        # (cheap) and already logged above at the full joint-state rate.
        _now = time.monotonic()
        if _now - self._last_robot_viz_t >= self._viz_min_interval_s:
            self._last_robot_viz_t = _now
            self._log_robot_pose(link_poses)
            self._log_cameras(cam_snapshots)

    def _log_robot_pose(self, link_poses: dict[str, np.ndarray]) -> None:
        """Log world-space link transforms from a pre-computed FK snapshot.

        Accepts a ``{link_name: 4*4_pose}`` dict built inside the model lock
        and called outside it, so Rerun I/O never delays FK computation.
        """
        for link, pose in link_poses.items():
            rr.log(
                f"robot/{link}",
                rr.Transform3D(translation=pose[:3, 3], mat3x3=pose[:3, :3], axis_length=0.03),
            )

    def _log_cameras(self, cam_snapshots: list[tuple]) -> None:
        """Log camera extrinsics (world-space Transform3D) from a pre-computed FK snapshot.

        Accepts a list of ``(name, world_T_cam)`` tuples built inside the model
        lock and called outside it, so Rerun I/O never delays FK computation.
        Intrinsics (``Pinhole``) are logged once as static data from
        ``_on_camera_info`` and are not repeated here.
        """
        for cam_name, world_t_cam in cam_snapshots:
            rr.log(
                f"cameras/{cam_name}",
                rr.Transform3D(translation=world_t_cam[:3, 3], mat3x3=world_t_cam[:3, :3]),
            )

    # -- cameras -------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo, *, cam_name: str) -> None:
        """Cache the 3x3 intrinsic matrix, image resolution, and optical frame ID.

        The ``Pinhole`` intrinsics are static (they never change with joint positions)
        and are logged here once with ``static=True`` so they are not re-sent on
        every joint-state callback.
        """
        k = np.array(msg.k, dtype=float).reshape(3, 3)
        self._cam_info[cam_name] = (k, msg.width, msg.height, msg.header.frame_id)
        rr.log(
            f"cameras/{cam_name}",
            rr.Pinhole(image_from_camera=k, resolution=[msg.width, msg.height], camera_xyz=self._camera_xyz),
            static=True,
        )

    def _on_image(self, msg: Image, *, cam_name: str) -> None:
        """Decode and log an incoming camera image, rate-gated to ``viz_max_hz``."""
        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        img, color_model = image_to_numpy(msg)
        if img is None:
            return
        _now = time.monotonic()
        if _now - self._last_image_viz_t.get(cam_name, 0.0) < self._viz_min_interval_s:
            return
        self._last_image_viz_t[cam_name] = _now
        rr.set_time("ros_time", timestamp=np.datetime64(stamp_ns, "ns"))
        rr.log(f"cameras/{cam_name}", rr.Image(img, color_model=color_model))

    # -- forward commands (action plot) --------------------------------
    def _on_forward_commands(self, msg: Float64MultiArray) -> None:
        """Log each commanded joint position scalar under ``action/position/<joint>``."""
        data = list(msg.data)
        n = min(len(self._cmd_joint_order), len(data))
        for i in range(n):
            rr.log(f"action/position/{self._cmd_joint_order[i]}", rr.Scalars(float(data[i])))

    # -- action chunk (predicted 3D trajectory) ------------------------
    def _on_action_chunk(self, msg: JointTrajectory) -> None:
        """Convert a predicted action-chunk trajectory into a 3D end-effector path via FK.

        Each waypoint in the ``JointTrajectory`` is run through the dedicated trajectory
        pinocchio model to obtain the gripper position in world space.  The resulting
        path is rendered as a fading ``rr.LineStrips3D`` / ``rr.Points3D`` overlay in
        the 3D scene, with alpha decreasing toward the chunk horizon.
        """
        if not msg.points or not msg.joint_names:
            return
        with self._model_lock:
            model = self._traj_model
            if model is None or not model.has_frame(self._ee_frame):
                return
            points: list[np.ndarray] = []
            for pt in msg.points:
                name_to_pos = {
                    name: float(pt.positions[i]) for i, name in enumerate(msg.joint_names) if i < len(pt.positions)
                }
                model.set_joint_positions(name_to_pos)
                pose = model.frame_pose(self._ee_frame)
                if pose is not None:
                    points.append(pose[:3, 3].copy())

        if len(points) < 1:
            return
        path = np.asarray(points)
        ts = stamp_to_datetime64(msg.header.stamp) if msg.header.stamp.sec else None
        if ts is not None:
            rr.set_time("ros_time", timestamp=ts)

        # Fade alpha from the gripper (opaque) to the chunk horizon (faint).
        r, g, b = self._traj_color
        alphas = np.linspace(230, 60, len(path)).astype(np.uint8)
        colors = np.column_stack([np.full(len(path), r), np.full(len(path), g), np.full(len(path), b), alphas])
        if len(path) >= 2:  # noqa: PLR2004
            rr.log("predicted_path", rr.LineStrips3D([path], colors=[[r, g, b, 180]], radii=0.002))
        rr.log("predicted_path/points", rr.Points3D(path, colors=colors, radii=0.004))


def main() -> None:
    """Entry point for the visualizer node.  Initializes ROS 2 and Rerun, builds the node, and spins."""
    rclpy.init()
    bootstrap = Node("pai_rerun_visualizer_params")

    def _declare(node: Node, name: str, default):
        node.declare_parameter(name, default)
        return node.get_parameter(name).value

    robot_description = str(_declare(bootstrap, "robot_description_topic", "/robot_description"))
    joint_states = str(_declare(bootstrap, "joint_states_topic", "/joint_states"))
    forward_commands = str(_declare(bootstrap, "forward_commands_topic", "/forward_position_controller/commands"))
    action_chunk = str(_declare(bootstrap, "action_chunk_topic", "/inference/action_chunk"))
    wrist_image = str(_declare(bootstrap, "wrist_image_topic", "/wrist_camera/image_raw"))
    wrist_info = str(_declare(bootstrap, "wrist_camera_info_topic", "/wrist_camera/camera_info"))
    static_image = str(_declare(bootstrap, "static_image_topic", "/static_camera/image_raw"))
    static_info = str(_declare(bootstrap, "static_camera_info_topic", "/static_camera/camera_info"))
    ee_frame = str(_declare(bootstrap, "ee_frame", "gripper_frame_link"))
    camera_xyz = str(_declare(bootstrap, "camera_xyz", "RDF"))
    cmd_joints = list(
        _declare(
            bootstrap,
            "cmd_joints",
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_flex_joint",
                "wrist_flex_joint",
                "wrist_roll_joint",
                "gripper_joint",
            ],
        )
    )
    traj_color = list(_declare(bootstrap, "trajectory_color", [0, 255, 0]))
    viewer = str(_declare(bootstrap, "viewer", "web"))
    memory_limit = str(_declare(bootstrap, "rerun_memory_limit", "1GB"))
    viz_max_hz = float(_declare(bootstrap, "viz_max_hz", 30.0))
    bootstrap.destroy_node()

    rr.init("pai_rerun_visualizer")
    if viewer == "native":
        rr.spawn(memory_limit=memory_limit)
    else:
        server_uri = rr.serve_grpc(server_memory_limit=memory_limit)
        rr.serve_web_viewer(connect_to=server_uri)

    blueprint = rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial3DView(name="3D Scene", origin="/"),
                rrb.Vertical(
                    rrb.Spatial2DView(name="Wrist", origin="cameras/cam_wrist"),
                    rrb.Spatial2DView(name="Overhead", origin="cameras/cam_static"),
                    row_shares=[1, 1],
                ),
                column_shares=[2, 1],
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(name="State (Joint Positions)", origin="state/position"),
                rrb.TimeSeriesView(name="Action (Commands)", origin="action/position"),
                column_shares=[1, 1],
            ),
            row_shares=[3, 1],
        ),
        auto_layout=False,
        auto_views=False,
    )
    rr.send_blueprint(blueprint)

    topics = Topics(
        robot_description=robot_description,
        joint_states=joint_states,
        forward_commands=forward_commands or None,
        action_chunk=action_chunk,
        cameras=[
            CameraCfg(name="cam_wrist", image_topic=wrist_image, info_topic=wrist_info),
            CameraCfg(name="cam_static", image_topic=static_image, info_topic=static_info),
        ],
    )

    node = VisualizerNode(
        topics,
        cmd_joint_order=cmd_joints,
        ee_frame=ee_frame,
        camera_xyz=camera_xyz,
        trajectory_color=(int(traj_color[0]), int(traj_color[1]), int(traj_color[2])),
        viz_max_hz=viz_max_hz,
    )

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
