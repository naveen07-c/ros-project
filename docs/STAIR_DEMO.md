# Robot.STEP walking and stairs

Run from this checkout:

```bash
cd "$HOME/Documents/ROS sim/ros-project"
./run_sim.sh
```

The default is now `demo`: an automatic 52-second kinematic demonstration. The robot walks along the ground, climbs four 40 mm risers with 360 mm treads, advances onto the landing and holds its final pose. Gazebo pause/play and reset controls govern playback; Ctrl+C exits. With no desktop display it runs headless. Override with `use_gui:=true` or `use_gui:=false`.

All 115 STEP part instances remain in the existing ten articulated meshes. Each foot lifts vertically, travels above the riser, then plants flat. Both feet reach each tread before the next climb, alternating the leading leg. Inverse kinematics uses the original URDF joint frames and limits. Sole heights use the actual CAD mesh bounds, including the small left/right difference.

**This is a kinematic animation, not autonomous or dynamically balanced stair climbing.** The demo removes the Physics system and prescribes link poses. It does not run ros2_control, publish sensor data, simulate contact forces, or send hardware commands. It validates visualization and planned foot geometry, not actuator capability, stability or self-collision freedom. CAD proportions remain fitted to the older simulation rig rather than dimensionally identical to the STEP assembly; see [CAD_VISUALS.md](CAD_VISUALS.md).

## Original physics modes

```bash
./run_sim.sh stand
./run_sim.sh walk
./run_sim.sh gazebo world:="$PWD/src/biped_bringup/worlds/stairs_world.sdf"
```

The last command loads the stairs with physics and the original robot. It does **not** activate a stair-climbing controller. The optional animation preview does not use dynamic force balance. Existing controller sources, collision shapes, inertias and joints are unchanged. The static stairs include collision boxes and friction for future controller work.

## Build and verify

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 tools/check_stair_demo.py
python3 tools/check_visual_only.py
```

The new playback system needs Gazebo Harmonic development headers (`gz-sim8`); the generator needs Xacro and NumPy. No CAD conversion package is needed at runtime.

The geometric check covers all 3,120 frames: IK/FK agreement, level soles, joint position and velocity limits, CAD foot bounds clearing every riser, a stationary grounded support foot, and both feet finishing at x=2.40 m and z=0.16 m. Peak joint speed is 2.826 rad/s (limit 3). Gazebo headless playback and GUI rendering were also exercised through completion. These are kinematic checks only.

Playback follows simulation time and holds the final frame without teleporting back. Each launch generates its world and frame table in a separate temporary directory, removed at shutdown.

Implementation: `src/biped_bringup/scripts/cad_demo.py` generates the path and articulated poses; `src/biped_bringup/src/cad_playback.cpp` applies them in Gazebo; `src/biped_bringup/worlds/stairs_world.sdf` supplies the reusable environment. Terrain changes must match the path constants; the geometry check detects mismatches.
