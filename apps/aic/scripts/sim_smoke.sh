#!/bin/bash
# Simulator smoke check, run on the host with the simulator up: the six arm
# joints publish, each camera publishes, and the contract loads against the
# built workspace. Headless rendering without a GPU is slow, so camera waits
# are long.

set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOINT_TIMEOUT_S="${AIC_SMOKE_JOINT_TIMEOUT_S:-120}"
CAMERA_TIMEOUT_S="${AIC_SMOKE_CAMERA_TIMEOUT_S:-180}"

echo "sim_smoke: waiting up to ${JOINT_TIMEOUT_S}s for /joint_states"
names="$(timeout "$JOINT_TIMEOUT_S" ros2 topic echo /joint_states sensor_msgs/msg/JointState --once --field name)"
for joint in shoulder_pan_joint shoulder_lift_joint elbow_joint wrist_1_joint wrist_2_joint wrist_3_joint; do
  if ! grep -q "$joint" <<< "$names"; then
    echo "sim_smoke: $joint missing from /joint_states: $names" >&2
    exit 1
  fi
done
echo "sim_smoke: arm joints present"

for cam in left center right; do
  echo "sim_smoke: waiting up to ${CAMERA_TIMEOUT_S}s for /${cam}_camera/image"
  timeout "$CAMERA_TIMEOUT_S" ros2 topic echo "/${cam}_camera/image" sensor_msgs/msg/Image --once --field width > /dev/null
done
echo "sim_smoke: cameras publish"

python - "$APP_ROOT" <<'PY'
import sys
from rosetta.contract.schema import load_contract
c = load_contract(f"{sys.argv[1]}/pai_aic/config/rosetta/aic.yaml")
print(f"sim_smoke: contract loads, fps={c.fps}")
PY
