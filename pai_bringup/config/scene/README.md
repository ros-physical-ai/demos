# Scene Configuration

Scene poses for table, tray, cubes, and arm base. Single source of truth for MuJoCo sim, Gazebo sim, and RViz.

## Updating Poses

Edit `poses.yaml` to change positions or orientations. All positions are in meters (x y z), orientations in radians (roll pitch yaw).

## Regenerating Scene Files

After editing `poses.yaml`, run the refresh utility to update the generated files.

From the repo root:

```bash
python3 pai_bringup/scripts/refresh_scene.py
```

With Pixi: `pixi run refresh-scene` (or run the command above inside `pixi shell`).

This updates:

- `mjcf/scene.xml` – MuJoCo simulation scene
- `mjcf/so_arm101.xml` – arm base position
- `urdf/so_arm101_mujoco.urdf.xacro` – default pose args for RViz

Gazebo world generation from this config is planned.

Then rebuild and relaunch as needed.
