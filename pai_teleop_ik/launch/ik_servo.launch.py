#!/usr/bin/env python3

# Copyright (C) 2026 Franco Cipollone
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

"""Standalone launch for the topic-driven IK servo node.

Included by the teleop frontend demos, or run on its own and driven by any
publisher of ``target_pose`` (PoseStamped) and ``gripper_command`` (Float64).

The servo is configured from a YAML ``params_file`` (``arm_joints``, ``ee_frame``,
topics, IK tuning, ...). The servo is arm-agnostic, so ``arm_joints`` must be
provided by that file (see ``pai_teleop_ik``'s README for the parameter list).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *_args, **_kwargs):
    # Only pass the params file if one was provided, so the node does not choke
    # on an empty path. ``arm_joints`` lives in the file (a proper string array),
    # which is why no per-parameter launch arguments are needed here.
    params_file = LaunchConfiguration("params_file").perform(context)
    parameters = []
    if params_file:
        parameters.append(params_file)
    parameters.append({"use_sim_time": LaunchConfiguration("use_sim_time")})

    servo = Node(
        package="pai_teleop_ik",
        executable="ik_servo_node",
        name="ik_servo",
        output="both",
        parameters=parameters,
    )
    return [servo]


def generate_launch_description():
    """Generate the IK servo launch description."""
    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value="",
            description="YAML params file for the ik_servo node (arm_joints, ee_frame, topics, IK tuning).",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]
    return LaunchDescription([*args, OpaqueFunction(function=_launch_setup)])
