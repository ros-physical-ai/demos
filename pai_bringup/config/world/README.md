# World Configuration

World poses for table, tray, cubes, and arm base. Single source of truth for MuJoCo sim, Gazebo sim, and RViz.

## Updating Poses

Edit `poses_args.xacro` to change positions or orientations. All positions are in meters (x y z), orientations in radians (roll pitch yaw).

## MuJoCo: Launch-Time Generation

MuJoCo scene XMLs are generated at launch time from xacro. Poses come from defaults in `poses_args.xacro`.

### Flow

1. Launch runs xacro on `mjcf/scene.xml.xacro` and `mjcf/so_arm101.xml.xacro` (poses from poses_args.xacro defaults)
2. Generated XMLs are written to a temp directory
3. The path to the generated `scene.xml` is passed to the URDF as `mujoco_model`
4. URDF xacro includes poses_args.xacro for RViz visualization

### Changing Poses

Edit `poses_args.xacro`, then rebuild and relaunch. With `--symlink-install`, relaunch alone suffices after editing.

### File Roles

- `poses_args.xacro`: Single source of truth; arg declarations with defaults for all poses
- `scene.xml.xacro`: World (table, tray, cubes) template; includes `so_arm101.xml`
- `so_arm101.xml.xacro`: Robot model and arm base pose template

### Adding New Shapes

Add the matching args to `poses_args.xacro`. Add the body/world element to `scene.xml.xacro` (and URDF, Gazebo when added). No launch code changes needed.

## Gazebo

Gazebo world generation from this config is planned.
