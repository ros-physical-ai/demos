# pai_teleop_ik

Topic-driven Cartesian differential inverse kinematics **servo** for any arm.

The `ik_servo_node` owns the whole robot-facing pipeline: it builds a Pinocchio
model from `/robot_description`, seeds the [Pink](https://github.com/stephane-caron/pink)
differential IK solver from `/joint_states`, and at a fixed rate drives the
tool frame toward a Cartesian target received on a topic, streaming joint
positions to a `position_controllers/JointGroupPositionController`.

Any input source (an RViz interactive marker, a phone, a leader arm, a scripted
trajectory, a bag replay) drives it by publishing a `geometry_msgs/PoseStamped`
target — no Pinocchio or Pink code required in the frontend.

## Architecture

```
input adapter ──ik_servo/target_pose (PoseStamped)──► ik_servo_node ──commands (Float64MultiArray)──► forward_position_controller
              ──ik_servo/gripper_command (Float64)──►  (Pink IK)    ◄── /joint_states + /robot_description
              ◄──ik_servo/ee_pose (PoseStamped)────────┘
```

The servo is robot-agnostic: the actuated joints, tool frame, and gripper are
all parameters. The included frontends are
[`pai_rviz_teleop`](../pai_rviz_teleop) and [`pai_phone_teleop`](../pai_phone_teleop).

## Topic contract

| Topic                      | Dir | Type                         | Notes                                           |
| -------------------------- | --- | ---------------------------- | ----------------------------------------------- |
| `/robot_description`       | in  | `std_msgs/String` (latched)  | builds the solver at startup                    |
| `/joint_states`            | in  | `sensor_msgs/JointState`     | seed-once + gripper feedback                    |
| `ik_servo/target_pose`     | in  | `geometry_msgs/PoseStamped`  | Cartesian target; `frame_id` resolved via model |
| `ik_servo/gripper_command` | in  | `std_msgs/Float64`           | gripper setpoint (rad); servo clamps to URDF    |
| `ik_servo/ee_pose`         | out | `geometry_msgs/PoseStamped`  | current commanded tool pose, in `root_frame`    |
| `<command_topic>`          | out | `std_msgs/Float64MultiArray` | arm joints + gripper, controller order          |

The `target_pose` / `gripper_command` / `ee_pose` topics are namespaced under
the node name (`ik_servo/`) by default; override the `*_topic` parameters to
change them.

### Frame handling

An incoming `target_pose` is interpreted in its `header.frame_id` and
transformed into the solver's universe frame (the URDF root link) using the
Pinocchio model's frame placements, cached per `frame_id`. For a fixed-base arm
these transforms are constant — no `tf2` is involved. An unknown `frame_id`
produces a one-time warning and is treated as already being in `root_frame`.
`ee_pose` is published in `root_frame` so adapters can publish targets back in
the same frame they receive.

## Running standalone

The servo can run on its own and be driven by any target publisher. `arm_joints`
is required (the servo has no default arm):

```bash
ros2 launch pai_teleop_ik ik_servo.launch.py \
  arm_joints:=shoulder_pan_joint,shoulder_lift_joint,elbow_flex_joint,wrist_flex_joint,wrist_roll_joint
# then, from another terminal:
ros2 topic pub --once ik_servo/target_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 0.2, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}'
```

## Retargeting another arm

Override the joint/frame parameters; the IK is arm-agnostic:

```bash
ros2 launch pai_teleop_ik ik_servo.launch.py arm_joints:=j1,j2,j3 ee_frame:=my_tool_link root_frame:=base_link
```

Set `arm_joints` (required), `gripper_joint`, `ee_frame`, `root_frame`, and
`command_topic` to match your robot. Every joint not in `arm_joints` is locked at
neutral, so the solver only actuates the joints you list.

## Parameters

| Parameter                 | Default                                 | Description                                                                        |
| ------------------------- | --------------------------------------- | ---------------------------------------------------------------------------------- |
| `arm_joints`              | **(required)**                          | Ordered joints the IK actuates; all others are locked. No default — set per arm.   |
| `gripper_joint`           | `gripper_joint`                         | Joint appended to commands from `gripper_command` (not part of IK). Empty to omit. |
| `ee_frame`                | `gripper_frame_link`                    | Tool frame driven toward the target.                                               |
| `root_frame`              | `world`                                 | Frame `ee_pose` is published in; default frame for unstamped targets.              |
| `control_rate`            | `50.0`                                  | IK / command streaming rate (Hz).                                                  |
| `position_cost`           | `0.5`                                   | FrameTask position weight.                                                         |
| `orientation_cost`        | `0.1`                                   | FrameTask orientation weight.                                                      |
| `posture_cost`            | `1e-3`                                  | PostureTask regularization weight.                                                 |
| `lm_damping`              | `1e-2`                                  | FrameTask Levenberg-Marquardt damping; raise to reduce near-singular oscillation.  |
| `max_joint_velocity`      | `2.0`                                   | Per-joint velocity cap (rad/s) enforced as a QP constraint.                        |
| `max_joint_acceleration`  | `10.0`                                  | Per-joint acceleration cap (rad/s²) enforced as a QP constraint.                   |
| `inactivity_timeout`      | `0.3`                                   | Seconds of target inactivity after which the arm holds and stops republishing.     |
| `target_lowpass_alpha`    | `0.5`                                   | SE3 low-pass per-tick geodesic fraction, `(0, 1]`; `1.0` disables filtering.       |
| `qp_solver`               | `quadprog`                              | QP backend used by Pink.                                                           |
| `command_topic`           | `/forward_position_controller/commands` | JointGroupPositionController command topic.                                        |
| `target_pose_topic`       | `ik_servo/target_pose`                  | Cartesian target input topic.                                                      |
| `gripper_command_topic`   | `ik_servo/gripper_command`              | Gripper setpoint input topic.                                                      |
| `ee_pose_topic`           | `ik_servo/ee_pose`                      | Commanded tool pose output topic.                                                  |
| `joint_states_topic`      | `/joint_states`                         | Measured joint state input topic.                                                  |
| `robot_description_topic` | `/robot_description`                    | Latched URDF input topic.                                                          |

## Reusable modules

Besides the node, the package exposes frontend-agnostic helpers:

- `pai_teleop_ik.ik_solver.DifferentialIKSolver` — the robot-agnostic Pink/Pinocchio solve.
- `pai_teleop_ik.se3_filter.se3_lowpass` — SE3 geodesic low-pass filter.
- `pai_teleop_ik.ros_utils` — `transient_local_qos`, `wait_for_robot_description`.
