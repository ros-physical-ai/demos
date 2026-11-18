#!/bin/bash
# Activation script for pixi.toml.
# Sources install/setup.bash when it exists (i.e., after a build).

eval "$(register-python-argcomplete ros2)" > /dev/null

if [ -f "${PIXI_PROJECT_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${PIXI_PROJECT_ROOT}/install/setup.bash"
fi

# The simulator container's zenoh router runs with shared memory disabled and
# the two sides do not share /dev/shm, so host peers must match.
export ZENOH_CONFIG_OVERRIDE='transport/shared_memory/enabled=false'

# Session config that the packaged libzenohc accepts; see the file header.
export ZENOH_SESSION_CONFIG_URI="${PIXI_PROJECT_ROOT}/config/zenoh/session.json5"
