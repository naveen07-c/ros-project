# CAD visuals for the unchanged simulation

> This document records the original visual-only work. The default runner now starts the [walking and stair demonstration](STAIR_DEMO.md); use `./run_sim.sh stand` for the standing behavior described below.

This checkout is in `Documents/ROS sim/ros-project`, with a byte-identical copy of the supplied `Robot.STEP` included at the repository root. It was cloned from the existing checkout of `https://github.com/naveen07-c/ros-project.git` at commit `8d6433f` (2026-09-26). GitHub fetch required authentication, so this is **not verified against the latest remote revision**. The original checkout in `Documents/ROS/ros-project` was left untouched.

## Run

```bash
cd "$HOME/Documents/ROS sim/ros-project"
./run_sim.sh
```

This starts the existing standing stack in isolated ROS domain 68 and Gazebo partition `ros_sim_cad` (override either with an environment variable). It uses a GUI when `DISPLAY` or `WAYLAND_DISPLAY` is available; otherwise it starts headless. Standing uses the original 15-degree crouch. To use the existing walking stack:

```bash
./run_sim.sh walk
```

Additional original launch arguments work, e.g. `./run_sim.sh stand use_gui:=false with_payload:=false`. For a fresh build, source `/opt/ros/jazzy/setup.bash` and run `colcon build --symlink-install` from this directory. CAD conversion dependencies are **not needed** to build or run; the mesh assets are already included.

## Appearance and dimensional limitation

All 115 CAD part instances are exported into 10 articulated mesh assets: the actual plate, brackets, servo bodies, shafts, screws, and foot plates. No generated replacement robot or decorative head is used. The original STEP RGB finish is preserved in Gazebo. URDF color input compensates for sdformat's 1.25 color gain; RViz may consequently display a darker shade.

**This is a fitted CAD appearance, not a dimensionally exact CAD robot.** The source assembly is approximately 282 mm tall, with different joint spacing from the simulation (600 mm standing pelvis height). The visual meshes are resized to the existing hip/knee/ankle locations so the simulation's physics stays unchanged. X is scaled by 2, Y maps the 99.029 mm CAD leg spacing to 200 mm, and Z is fitted between the sole, ankle mounting plane, knee, and hip landmarks. Exact transforms and part assignments are recorded in `src/biped_description/meshes/cad/manifest.json`.

The simulation has co-located ankle pitch and roll axes, unlike the separated CAD geometry. Both CAD ankle subassemblies therefore move with each foot. Visual pivots and proportions are an adaptation to the existing rig, not a new CAD-derived kinematic model. The visible foot plates extend beyond the original collision soles. Collision geometry has deliberately not been resized.

The STEP has no separate payload/head object. The old head/chest placeholders were removed visually. The original payload link, its 1 kg mass, collision, and `with_payload` behavior remain in the physics model, even though no payload is visible.

## Verification

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/check_visual_only.py
```

The check compares this model against Git commit `8d6433f` after removing only visual/material elements, for both payload modes, in expanded URDF and Gazebo SDF. All joints, limits, axes, masses, inertias, collision shapes, contact settings, sensors, and control plugins must match. Controller sources, gait/controller YAML, and the original flat world must also be unchanged. The separate stair world is an addition.

The three ROS packages built successfully. Original and CAD versions both completed the standing goal, with both controllers active. Measured settled pelvis heights were 0.5855283745 m (original) and 0.5855283670 m (CAD). The existing walking algorithm has not been retuned or newly validated for a longer walking distance. Existing hardware-code compiler warnings and SDF contact warnings were left unchanged.

Gazebo rendering was inspected with its Ogre renderer on a virtual X display; the default Ogre2 renderer failed to initialize on that virtual display. This QA workaround does not change the project's default renderer or physics. `gazebo-cad.png` is the resulting Gazebo preview; `cad-original.png` is a rendering of the original STEP before fitting.

## Regenerate meshes

Use a separate environment with `cadquery-ocp==8.0.1.1.0` and NumPy, then:

```bash
python tools/export_cad_visuals.py Robot.STEP
```

The exporter keeps STEP assembly placements and uses 0.08 mm tessellation deflection and 0.25 rad angular tolerance. It records the source SHA-256 and requires every CAD instance to be assigned exactly once. Mesh coordinates are exported in metres. Do not use these visual meshes for collisions or recompute inertias from them if preserving the original simulation behavior.
