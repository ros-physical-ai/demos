#!/bin/bash
# Drift guard: the AIC message package built on the host must match the one
# installed in the simulator image, or ROS 2 type hashes differ and the
# action topic silently never connects.
#
# Compares the generated type-description files (the input to the type
# hash) for every interface in aic_control_interfaces. Needs the image
# pulled, the host workspace built and the environment activated.

set -euo pipefail

APP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG=aic_control_interfaces
IMAGE="$(docker compose -f "$APP_ROOT/compose.yaml" config --images | head -1)"
HOST_SHARE="$(ros2 pkg prefix "$PKG")/share/$PKG"
IMAGE_SHARE="/ws_aic/install/share/$PKG"

if [ -z "$IMAGE" ]; then
  echo "check_interfaces: no image in $APP_ROOT/compose.yaml" >&2
  exit 1
fi

host_files="$(cd "$HOST_SHARE" && find msg srv -name '*.json' 2>/dev/null | sort)"
image_files="$(docker run --rm --entrypoint bash "$IMAGE" -c \
  "cd $IMAGE_SHARE && find msg srv -name '*.json' 2>/dev/null | sort")"

if [ "$host_files" != "$image_files" ]; then
  echo "check_interfaces: interface sets differ" >&2
  diff <(echo "$host_files") <(echo "$image_files") >&2 || true
  exit 1
fi

status=0
for f in $host_files; do
  if diff -q <(docker run --rm --entrypoint cat "$IMAGE" "$IMAGE_SHARE/$f") "$HOST_SHARE/$f" >/dev/null; then
    echo "ok   $PKG/$f"
  else
    echo "DIFF $PKG/$f" >&2
    status=1
  fi
done

if [ "$status" -ne 0 ]; then
  echo "check_interfaces: host and image disagree; align aic.repos and the compose image digest" >&2
fi
exit "$status"
