#!/bin/bash
# Hide every AIC package from colcon except the ones the host builds.
# The simulator, controllers and their Gazebo stack come from the container
# image. An allowlist, so an upstream package addition cannot pull the
# simulator into the host build.
#
# Directories that are neither an allowlisted package nor on the path to one
# get a COLCON_IGNORE at the topmost level, which also hides packages that
# colcon would discover through setup.py or pyproject.toml alone.

set -euo pipefail

AIC_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/external/aic"
ALLOWLIST=(
  aic_control_interfaces
)

if [ ! -d "$AIC_ROOT" ]; then
  echo "prune_aic_source: $AIC_ROOT does not exist; run 'vcs import external < aic.repos' first" >&2
  exit 1
fi

# Resolve allowlisted package names to directories through their package.xml.
allowed_dirs=()
while IFS= read -r manifest; do
  pkg_name="$(sed -n 's|.*<name>\(.*\)</name>.*|\1|p' "$manifest" | head -1)"
  for allowed in "${ALLOWLIST[@]}"; do
    if [ "$pkg_name" = "$allowed" ]; then
      allowed_dirs+=("$(dirname "$manifest")")
    fi
  done
done < <(find "$AIC_ROOT" -name package.xml -not -path '*/.git/*')

if [ "${#allowed_dirs[@]}" -ne "${#ALLOWLIST[@]}" ]; then
  echo "prune_aic_source: expected ${#ALLOWLIST[@]} allowlisted package(s), found ${#allowed_dirs[@]}" >&2
  exit 1
fi

# A directory is kept when it is an allowlisted package, an ancestor of one,
# or inside one.
is_kept() {
  local dir="$1"
  for allowed in "${allowed_dirs[@]}"; do
    [ "$dir" = "$allowed" ] && return 0
    [[ "$allowed" == "$dir"/* ]] && return 0
    [[ "$dir" == "$allowed"/* ]] && return 0
  done
  return 1
}

find "$AIC_ROOT" -name COLCON_IGNORE -not -path '*/.git/*' -delete

hidden=0
ignored_prefixes=()
while IFS= read -r dir; do
  for prefix in "${ignored_prefixes[@]}"; do
    [[ "$dir" == "$prefix"/* ]] && continue 2
  done
  is_kept "$dir" && continue
  touch "$dir/COLCON_IGNORE"
  ignored_prefixes+=("$dir")
  hidden=$((hidden + 1))
done < <(find "$AIC_ROOT" -mindepth 1 -type d -not -path '*/.git' -not -path '*/.git/*' | sort)

echo "prune_aic_source: kept ${#allowed_dirs[@]} package(s), hid $hidden directories"
