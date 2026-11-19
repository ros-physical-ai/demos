#!/bin/bash
# Lists connected serial ports and cameras with the udev properties that
# pai_bringup/config/hardware/99-so-arm101.rules.example matches on.

set -u

prop() {
  # Prints one udev property of a device node, empty if unset.
  udevadm info -q property -n "$1" | sed -n "s/^$2=//p"
}

echo "=== Serial ports (arms) ==="
found=false
for dev in /dev/ttyACM* /dev/ttyUSB*; do
  [ -e "$dev" ] || continue
  found=true
  echo "$dev"
  echo "  ID_VENDOR_ID=$(prop "$dev" ID_VENDOR_ID)"
  echo "  ID_MODEL_ID=$(prop "$dev" ID_MODEL_ID)"
  echo "  ID_SERIAL_SHORT=$(prop "$dev" ID_SERIAL_SHORT)"
done
$found || echo "  none found"

echo ""
echo "=== Cameras ==="
found=false
for dev in /dev/video*; do
  [ -e "$dev" ] || continue
  found=true
  sys=/sys/class/video4linux/$(basename "$dev")
  index=$(cat "$sys/index")
  # A UVC camera exposes a video stream at index 0 and a metadata node at index 1.
  if [ "$index" = "0" ]; then
    kind="video stream"
  else
    kind="metadata, not usable as a camera"
  fi
  echo "$dev  [$kind]"
  echo "  name=$(cat "$sys/name")"
  echo "  ID_PATH=$(prop "$dev" ID_PATH)"
done
$found || echo "  none found"

echo ""
echo "=== Symlinks from udev rules ==="
for link in /dev/so101_leader /dev/so101_follower /dev/cam_wrist /dev/cam_static; do
  if [ -e "$link" ]; then
    echo "$link -> $(readlink -f "$link")"
  else
    echo "$link  (missing)"
  fi
done
