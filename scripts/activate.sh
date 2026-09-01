#!/bin/bash
# Safe activation script for pixi.toml.
# Sources install/setup.bash when it exists (i.e., after a build).
# This avoids pixi warnings about missing activation scripts on fresh checkouts.

eval "$(register-python-argcomplete ros2)" > /dev/null

# The base demo builds into install/; the AIC env builds into install_aic/ so the
# two can coexist. Pick the tree from the active pixi environment: the `aic` env
# uses install_aic/, everything else uses install/. We key off PIXI_ENVIRONMENT_NAME
# (set by pixi core *before* this script runs) rather than a [feature.*.activation.env]
# var, because those feature env vars are applied AFTER activation scripts run and so
# aren't visible here. PAI_INSTALL_DIR still works as an explicit override if set.
if [ -n "${PAI_INSTALL_DIR}" ]; then
  _pai_install_dir="${PAI_INSTALL_DIR}"
elif [ "${PIXI_ENVIRONMENT_NAME}" = "aic" ]; then
  _pai_install_dir="install_aic"
else
  _pai_install_dir="install"
fi
_pai_install="${PIXI_PROJECT_ROOT}/${_pai_install_dir}"
unset _pai_install_dir

if [ -f "${_pai_install}/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${_pai_install}/setup.bash"

  # The from-source gz stack installs its command configs (sim9.yaml, gui9.yaml,
  # etc.) into each package's isolated install/<pkg>/share/gz dir, but — unlike
  # aic_bringup — the upstream gz packages ship no GZ_CONFIG_PATH env hook, so
  # `gz` (install/gz-tools2/bin/gz) never finds them and only lists `sdf` (the
  # sole config in the conda share/gz). Register every workspace gz config dir.
  for _gz_cfg_dir in "${_pai_install}"/*/share/gz; do
    [ -d "${_gz_cfg_dir}" ] && export GZ_CONFIG_PATH="${_gz_cfg_dir}:${GZ_CONFIG_PATH}"
  done
  unset _gz_cfg_dir
fi
unset _pai_install
