#!/bin/bash
# Script to move SO-ARM100 through various positions
# Each position is held for 4 seconds before moving to the next

SLEEP_TIME=4 # seconds to hold each position
RATE=20 # publish rate in Hz

echo "Starting arm demo sequence..."
echo "Press Ctrl+C to stop"

# Home position (all zeros)
echo "Moving to: Home position"
timeout $SLEEP_TIME ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --rate $RATE

# Slight rotation and tilt
echo "Moving to: Slight rotation and tilt"
timeout $SLEEP_TIME ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.2, -0.4, 0.0, 0.0, 0.0, 0.4]}" --rate $RATE

# Reach forward low
echo "Moving to: Reach forward low"
timeout $SLEEP_TIME ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.0, -0.8, 0.5, -0.3, 0.0, 0.0]}" --rate $RATE

# Reach forward high
echo "Moving to: Reach forward high"
timeout $SLEEP_TIME ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.0, -0.3, -0.5, 0.2, 0.0, 0.0]}" --rate $RATE

# Rotate left and extend
echo "Moving to: Rotate left and extend"
timeout $SLEEP_TIME ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.8, -0.5, 0.3, 0.0, 0.2, 0.5]}" --rate $RATE

# Rotate right and extend
echo "Moving to: Rotate right and extend"
timeout $SLEEP_TIME ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [-0.8, -0.5, 0.3, 0.0, -0.2, 0.5]}" --rate $RATE

# Gripper open (last joint)
echo "Moving to: Gripper open"
timeout $SLEEP_TIME ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.0, -0.4, 0.2, 0.0, 0.0, 1.0]}" --rate $RATE

# Gripper closed
echo "Moving to: Gripper closed"
timeout $SLEEP_TIME ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.0, -0.4, 0.2, 0.0, 0.0, -0.5]}" --rate $RATE

# Return to home
echo "Moving to: Home position (final)"
timeout $SLEEP_TIME ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --rate $RATE

echo "Demo sequence complete!"
