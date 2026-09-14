#!/usr/bin/env bash
# Execute a command with the colcon overlay's lib dirs on the dynamic linker
# search path.
#
# Why this exists: macOS's system /bin/bash is SIP-protected, and dyld strips
# DYLD_* environment variables (incl. DYLD_LIBRARY_PATH) from any process
# spawned through it — including pixi's own task shell. That silently breaks
# dlopen()-by-name lookups such as ROS 2's typesupport loading for
# custom-built message packages (e.g. rosetta_interfaces,
# mujoco_ros2_control_msgs), which aren't found via rpath the way
# conda-managed libraries are.
#
# AMENT_PREFIX_PATH is not a DYLD_* variable, so it survives. Rebuild
# DYLD_LIBRARY_PATH from it here, after the shell has already stripped the
# original, then exec the requested command. No-op on Linux.
set -euo pipefail

if [ "$(uname)" = "Darwin" ]; then
  DYLD_LIBRARY_PATH=""
  for prefix in $(printf '%s' "${AMENT_PREFIX_PATH:-}" | tr ':' '\n'); do
    if [ -d "$prefix/lib" ]; then
      DYLD_LIBRARY_PATH="$prefix/lib:${DYLD_LIBRARY_PATH}"
    fi
  done
  export DYLD_LIBRARY_PATH="${DYLD_LIBRARY_PATH%:}"
fi

exec "$@"
