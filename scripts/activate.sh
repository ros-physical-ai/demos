#!/bin/bash
# Safe activation script for pixi.toml.
# Sources install/setup.bash when it exists (i.e., after a build).
# This avoids pixi warnings about missing activation scripts on fresh checkouts.

if [ -f "${PIXI_PROJECT_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${PIXI_PROJECT_ROOT}/install/setup.bash"
fi
