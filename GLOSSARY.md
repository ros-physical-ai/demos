# ROS Physical AI Demos

Vocabulary shared by the applications in this repository and the Record →
Train → Deploy pipeline they run. Each application may add terms of its own.

## Language

### Repository

**Application**:
A self-contained Pixi workspace under `apps/<name>/` with its own manifest,
lockfile, ROS 2 distribution and build tree.
_Avoid_: demo, environment, workspace, app env

**Pipeline**:
The Record → Train → Deploy sequence: record episodes to bags, port them to a
LeRobot dataset, train a policy, run the policy against the robot.
_Avoid_: workflow, learning loop

**Contract**:
The Rosetta YAML that maps ROS 2 topics to LeRobot dataset features. One
contract serves recording, porting and deployment of a given robot.
_Avoid_: config, schema, mapping

**Episode**:
One recorded demonstration, bounded by a start and a stop command to the
recorder.
_Avoid_: run, trial, demo

### AIC application

**Simulator container**:
The published `aic_eval` image running Gazebo, the AIC controller, the AIC
adapter and the zenoh router. Supplies the simulated robot to the host.
_Avoid_: docker, eval, the sim, aic_eval service

**Host**:
The `apps/aic` Pixi environment where every process of ours runs: recorder,
teleop, porter, trainer and policy runner.
_Avoid_: native, local, pixi side

**Thin build**:
The host colcon build limited to Rosetta, `pai_aic` and the allowlisted AIC
packages. Excludes every simulator package.
_Avoid_: partial build, light build

**Drift guard**:
The check that the AIC interface definitions built on the host are identical
to those installed in the simulator container.
_Avoid_: interface check, version check

**Scene**:
The task board rail configuration and cable placement the robot works
against. One scene per recording session.
_Avoid_: world, trial, config

**Twist action**:
The 6-dimensional Cartesian velocity command, linear then angular, expressed
in the tool frame and sent to the AIC controller. The only action the policy
produces.
_Avoid_: pose command, motion update, end-effector command

**Tool frame**:
The `gripper/tcp` frame at the gripper's tool center point. Every twist action
is expressed in it.
_Avoid_: end-effector frame, TCP frame, gripper frame

**Scene reset**:
Returning the arm to its home joints and respawning the cable in the gripper
so the next episode starts from the scene's initial state.
_Avoid_: reset, respawn, re-spawn scene
