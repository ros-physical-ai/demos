### Pick and Place with SO-ARM and LeRobot

#### Setup

The demo was developed on a desktop with a RTX 5090 card which runs the NVIDIA blackwell architecture.
The `570` driver needs to be installed (had issues with 580). This should install `cuda 12.8`.
The instruction set for this architecture `sm_120` is not supported on the default `pytorch` version.

The desired end state of dependencies is as follows.

```bash
$ pip freeze | grep torch
torch==2.7.0+cu128
torchaudio==2.7.0+cu128
torchcodec==0.4.0
torchvision==0.22.0+cu128

```
And running these commands should produce the result below without any `sm_120` errors.
```bash
$ python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.device_count()); print(torch.cuda.get_device_name(0))"
True
1
NVIDIA GeForce RTX 5090
```

To achieve this, it's strongly recommended to rely on `conda` to manage a virtual environment with python dependencies. Follow install instructions on `lerobot` closely.

In addition, ensure to install the exact versions of pip packages above.

```
# create conda enviroment for ROS 2 and Lerobot
conda create -n lerobot_ros2 -c conda-forge -c robostack-kilted ros-kilted-desktop
conda activate lerobot_ros2
conda install ffmpeg -c conda-forge
pip install torch==2.7.0+cu128 torchaudio==2.7.0+cu128 torchcodec==0.4.0 torchvision==0.22.0+cu128 --index-url https://download.pytorch.org/whl/cu128
pip install lerobot==0.3.3
```

#### Calibration

```bash
cp config/lerobot/follower_arm.json ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/follower_arm.json
cp config/lerobot/leader_arm.json ~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/leader_arm.json
```

#### Teleoperate

```bash
python3 src/lerobot/teleoperate.py \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_arm
```

With camera

```bash
python3 src/lerobot/teleoperate.py \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_arm \
    --robot.cameras="{overhead: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
    --display_data=true
```

#### Record

First, set your Hugging Face username automatically using the [Hugging Face CLI](https://huggingface.co/docs/huggingface_hub/en/guides/cli):
```bash
export HF_USER=$(hf auth whoami | head -n 1)
```

Or set it manually:
```bash
export HF_USER=your_username_here
```

Then run the record command:
```bash
python3 src/lerobot/record.py \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_arm \
    --robot.cameras="{overhead: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
    --display_data=true \
    --dataset.repo_id=${HF_USER}/record-test \
    --dataset.num_episodes=1 \
    --dataset.single_task="Pick up the blue cube" \
    --dataset.push_to_hub=false
```

#### Replay
```bash
python3 src/lerobot/replay.py \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm \
    --dataset.repo_id=${HF_USER}/record-test \
    --dataset.episode=0 # choose the episode you want to replay
```

#### Train
```bash
python3 src/lerobot/scripts/train.py \
    --dataset.repo_id=${HF_USER}/move_to_cube \
    --policy.type=act \
    --output_dir=outputs/train/act_move_to_cube \
    --job_name=act_move_to_cube \
    --policy.device=cuda \
    --wandb.enable=false \
    --policy.repo_id=${HF_USER}/move_to_cube_policy \
    --policy.push_to_hub=false
```

#### Evaluate
```bash
python3 src/lerobot/record.py \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM1 \
    --robot.id=follower_arm \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM0 \
    --teleop.id=leader_arm \
    --robot.cameras="{overhead: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
    --dataset.repo_id=${HF_USER}/eval_move_to_cube \
    --dataset.single_task="Pick up the blue cube" \
    --display_data=true \
    --policy.push_to_hub=false \
    --policy.path=outputs/train/act_move_to_cube/checkpoints/040000/pretrained_model
```

## Inference in Gazebo

```python
python3 pai_bringup/scripts/lerobot_inference_node --ros-args \
    -p policy_path:=huggingface/lerobot/outputs/train/act_move_to_cube_2/checkpoints/last/pretrained_model \
    -p camera_topic:=/camera \
    -p command_topic:=/forward_position_controller/commands \
    -p task:="Move to blue cube" \
    -p device:=cuda
```

### Parameters

- `policy_path`: Path to the trained LeRobot policy model (default: `outputs/train/act_move_to_cube/checkpoints/040000/pretrained_model`)
- `camera_topic`: ROS 2 topic for camera input (default: `/camera`)
- `command_topic`: ROS 2 topic for joint commands (default: `/forward_position_controller/commands`)
- `device`: PyTorch device for inference (default: `cuda`)
- `use_amp`: Use automatic mixed precision (default: `false`)
- `task`: Task description for the policy (default: `"Pick up the blue cube"`)
- `robot_type`: Robot type identifier (default: `so101_follower`)
- `fps`: Target inference frequency (default: `30.0`)
- `camera_width`: Expected camera image width (default: `640`)
- `camera_height`: Expected camera image height (default: `480`)
- `use_degrees_training`: Was the model trained with `use_degrees=True`? (default: `false`)

### Topic Interface

**Subscribed Topics:**
- `/camera` (or specified camera_topic): `sensor_msgs/Image` - Camera data for policy input

**Published Topics:**
- `/forward_position_controller/commands` (or specified command_topic): `std_msgs/Float64MultiArray` - Joint position commands

### Expected Message Format

The `Float64MultiArray` contains joint positions in the following order:
1. `shoulder_pan_joint`
2. `shoulder_lift_joint`
3. `elbow_joint`
4. `wrist_pitch_joint`
5. `wrist_roll_joint`
6. `jaw_joint`

## Implementation Notes

### Key Features
- **Real-time Inference**: Runs policy inference at specified FPS (default 30 Hz)
- **Image Processing**: Automatically converts ROS 2 `sensor_msgs/Image` to LeRobot format
- **Device Support**: Configurable PyTorch device (CUDA/CPU)
- **Rate Limiting**: Prevents overload by limiting inference frequency
- **Unit Conversion**: Automatically converts LeRobot action units to ROS 2 radians
- **Error Handling**: Robust error handling and logging

### CRITICAL: Unit Conversion
**LeRobot vs ROS 2 Unit Differences**

LeRobot SO101 training uses different units than ROS 2:

- **LeRobot Training Data**:
  - `use_degrees=False` (default): Actions normalized to `[-100, 100]` range
  - `use_degrees=True`: Actions directly in degrees
- **ROS 2 Commands**: Joint positions in **radians**

**The inference node automatically converts between these units**, but you **MUST**:

1. **Verify your training configuration**: Check if your model was trained with `use_degrees=True` or `False`
2. **Set the `use_degrees_training` parameter correctly**
3. **Verify joint limits**: The hardcoded joint limits are approximations and may not match your specific robot's calibration

**Joint Limits Used (in degrees):**
```python
joint_limits_deg = {
    'shoulder_pan_joint': (-180, 180),      # Full rotation
    'shoulder_lift_joint': (-90, 90),       # Shoulder lift range
    'elbow_joint': (-135, 135),              # Elbow range
    'wrist_pitch_joint': (-90, 90),         # Wrist pitch
    'wrist_roll_joint': (-180, 180),        # Wrist roll full rotation
    'jaw_joint': (0, 100)                   # Gripper 0-100%
}
```

** WARNING**: Incorrect unit conversion can cause unsafe robot movements! Always test with simulation first.

### Architecture
The inference node follows the same data processing pipeline as the original LeRobot recording script:

1. **Image Conversion**: ROS 2 Image → OpenCV → NumPy → PyTorch
2. **Preprocessing**: Normalization, resizing, channel reordering
3. **Policy Inference**: LeRobot policy.select_action()
4. **Command Publishing**: Action values → Float64MultiArray

### Camera Data Processing
- Images are converted from ROS 2 `sensor_msgs/Image` to OpenCV format using `cv_bridge`
- Images are resized to expected dimensions if needed
- Pixel values are normalized to [0,1] range
- Channel order is converted from HWC to CHW for PyTorch

### Policy Integration
- Uses the same `predict_action` logic as the original LeRobot code
- Supports both CUDA and CPU inference
- Maintains compatibility with all LeRobot policy types (ACT, Diffusion, etc.)

## Troubleshooting

### Common Issues

1. **Python Environment Conflicts**:
   - **Problem**: `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'` or numpy import errors
   - **Cause**: ROS 2 installed for system Python, but running from conda environment creates binary incompatibilities
   - **Solution**: Use `lerobot_mixed_env.py` or `lerobot_ros2_launcher.sh` for proper environment isolation

2. **Policy Loading Error**: Ensure the policy path is correct and accessible
3. **CUDA Out of Memory**: Reduce image resolution or switch to CPU device
4. **Topic Connection Issues**: Check topic names match between camera source and inference node
5. **Import Errors**: Ensure LeRobot is properly installed in the Python environment
6. **Permission Issues**: Make sure the script has execute permissions

## ROS 2 Controls commands

```bash
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray '{layout: {dim: [{label: joint, size: 6, stride: 1}]}, data: [0.2,-0.4,0,0,0,0.4]}' --rate 10
```
