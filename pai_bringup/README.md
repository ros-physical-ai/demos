# pai_bringup

Launch files for the SO-ARM101 in simulation and on real hardware.

## Gazebo

```bash
ros2 launch pai_bringup so_arm_gz_bringup.launch.py
```

## MuJoCo

```bash
ros2 launch pai_bringup so_arm_mujoco_bringup.launch.py
```

## Real hardware

```bash
ros2 launch pai_bringup so_arm_real_bringup.launch.py
```

## MoveIt

MoveIt requires the `joint_trajectory_controller`. Launch the bringup with RViz disabled, then start MoveIt separately.

### Simulation (Gazebo)

```bash
# Terminal 1
ros2 launch pai_bringup so_arm_gz_bringup.launch.py \
    initial_joint_controller:=joint_trajectory_controller launch_rviz:=false

# Terminal 2
ros2 launch pai_bringup so_arm_moveit.launch.py use_sim_time:=true
```

### Simulation (MuJoCo)

```bash
# Terminal 1
ros2 launch pai_bringup so_arm_mujoco_bringup.launch.py \
    initial_joint_controller:=joint_trajectory_controller launch_rviz:=false

# Terminal 2
ros2 launch pai_bringup so_arm_moveit.launch.py use_sim_time:=true
```

### Real hardware

```bash
# Terminal 1
ros2 launch pai_bringup so_arm_real_bringup.launch.py \
    initial_joint_controller:=joint_trajectory_controller launch_rviz:=false

# Terminal 2
ros2 launch pai_bringup so_arm_moveit.launch.py
```
