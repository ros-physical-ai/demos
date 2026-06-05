#!/bin/bash
# TODO(francocipollone): This script is a temporary workaround to reset the cubes positions in Gazebo. It should be removed once we have a better way to do this, e.g. by using a Gazebo plugin or by using ROS 2 services.
gz service -s /world/pai_world/set_pose   --reqtype gz.msgs.Pose   --reptype gz.msgs.Boolean   --req "name: 'cube_small', position: {x: 0.16, y: -0.11, z: 0.41}, orientation: {x: 0, y: 0, z: 0.0299955, w: 0.99955}" &
gz service -s /world/pai_world/set_pose   --reqtype gz.msgs.Pose   --reptype gz.msgs.Boolean   --req "name: 'cube_medium', position: {x: 0.17, y: 0.05, z: 0.41}, orientation: {w: 1.0}" &
gz service -s /world/pai_world/set_pose   --reqtype gz.msgs.Pose   --reptype gz.msgs.Boolean   --req "name: 'cube_large', position: {x: 0.12, y: 0.20, z: 0.41}, orientation: {x: 0, y: 0, z: -0.3569493, w: 0.9341238}" &
wait
