#!/bin/bash
# Safe activation script for pixi.toml.
# Sources install/setup.bash when it exists (i.e., after a build).
# This avoids pixi warnings about missing activation scripts on fresh checkouts.

eval "$(register-python-argcomplete ros2)" > /dev/null

if [ -f "${PIXI_PROJECT_ROOT}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${PIXI_PROJECT_ROOT}/install/setup.bash"

  # The from-source gz stack installs its command configs (sim9.yaml, gui9.yaml,
  # etc.) into each package's isolated install/<pkg>/share/gz dir, but — unlike
  # aic_bringup — the upstream gz packages ship no GZ_CONFIG_PATH env hook, so
  # `gz` (install/gz-tools2/bin/gz) never finds them and only lists `sdf` (the
  # sole config in the conda share/gz). Register every workspace gz config dir.
  for _gz_cfg_dir in "${PIXI_PROJECT_ROOT}"/install/*/share/gz; do
    [ -d "${_gz_cfg_dir}" ] && export GZ_CONFIG_PATH="${_gz_cfg_dir}:${GZ_CONFIG_PATH}"
  done
  unset _gz_cfg_dir
fi
