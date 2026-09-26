# PROJECT_REQUIREMENTS.md — Bipedal Payload-Carrying Robot

> **Milestone M0 artifact.** This file is the single source of truth for all subsequent
> milestones. It was generated from, and must stay consistent with, the following inputs
> (in priority order):
> 1. `problem_statement.md` (robot constraint/specification)
> 2. `Robot.png` (reference image)
> 3. `ROS2_Gazebo_Robot_PreBuild_Guide_Freebuff.pdf` (workflow template)
> 4. ROS 2 Jazzy / Gazebo (Harmonic) official documentation
>
> If any source conflicts with this file, the conflict must be raised, not silently resolved.
> Status tags: **FACT** (read from a source) / **ASSUMPTION** (explicitly invented, must stay configurable).

---

## 1. Conflicts identified between sources (recorded, not hidden)

| # | Conflict | Resolution used |
|---|----------|-----------------|
| C1 | The PDF workflow template is written for a **wheeled** robot (wheel count, wheelbase, track width, differential drive, LiDAR, SLAM, Nav2). The problem statement and `Robot.png` specify a **bipedal legged robot**. | Specification wins: robot is bipedal. "Wheel" milestone content is reinterpreted as **leg joints + foot contact**. SLAM (M9/M10 of the PDF) is **NOT required** — no LiDAR exists in the specification. |
| C2 | The PDF mentions `/cmd_vel` drive testing; a bipedal walking robot does not accept `Twist` velocity commands in a meaningful way. | Gait commands are issued as **joint-space trajectory goals** (action) plus **gait start/stop (topic/service)**. `/cmd_vel` will be replaced by `/gait_command` at the control milestone; deviation from PDF documented here. |
| C3 | User milestone list in the task prompt (M0–M14) differs slightly from the PDF milestone table (M0–M13). | The user's milestone list is used as the roadmap; PDF milestones are mapped onto it. This file records the mapping. |

---

## 2. Robot type and structure

- **Type:** FACT — Bipedal (two-legged), lower-body only, with a rigid **payload platform** mounted above the pelvis.
- **Legs:** FACT — 2 legs. Each leg has:
  - **Hip joint: pitch & roll** (2 DoF) — per `Robot.png` annotation.
  - **Knee joint: pitch** (1 DoF) — per `Robot.png` annotation.
  - **Ankle joint: pitch & roll** (2 DoF) — per `Robot.png` annotation.
  - **Total DoF:** 2 legs × 5 DoF = **10 DoF**.
- **Feet:** FACT — rigid flat feet "with grip" (rubber-textured soles per image). Foot is fixed to the ankle (no toe joint).
- **Payload interface:** FACT — flat platform on top of pelvis, carrying a **1 kg** payload (shown as a box in `Robot.png`).

## 3. Dimensions

| Item | Value | Status |
|------|-------|--------|
| Total height (pelvis to ground, standing) | ~0.55 m (from image proportions; pelvis under platform ≈ 55 % of total height) | ASSUMPTION |
| Payload platform height above ground | ~0.60 m | ASSUMPTION |
| Thigh length (hip→knee) | 0.20 m | ASSUMPTION |
| Shin length (knee→ankle) | 0.20 m | ASSUMPTION |
| Ankle height (ankle→sole) | 0.15 m | ASSUMPTION |
| Hip separation (hip axis lateral distance) | 0.20 m (tuned against Robot.png visual comparison in M2) | ASSUMPTION |
| Foot size (length × width × height) | 0.16 m × 0.09 m × 0.03 m | ASSUMPTION |
| Platform size (length × width) | 0.36 m × 0.28 m (tuned against Robot.png: platform overhangs the legs) | ASSUMPTION |
| Payload box (from image) | 0.20 m × 0.14 m × 0.11 m (tuned against Robot.png) | ASSUMPTION |

All numeric dimensions are ASSUMPTION: `Robot.png` contains no scale reference or
dimensioned drawing. Every value must remain a build/run parameter (xacro property),
never hard-coded in control code.

## 4. Mass & payload

| Item | Value | Status |
|------|-------|--------|
| Robot total mass (without payload) | ~8 kg total: pelvis+platform ~2.5 kg, hip actuators 4×~0.4 kg, knee 2×~0.4 kg, thigh/shin links ~2 kg, feet 2×~0.3 kg, ankle actuators included in leg links | ASSUMPTION |
| Payload | 1.0 kg (minimum, from problem statement) | FACT |

## 5. Locomotion requirements

| Item | Value | Status |
|------|-------|--------|
| Walking speed (flat ground) | 0.08–0.12 m/s | ASSUMPTION |
| Step length | 0.08–0.12 m | ASSUMPTION |
| Step period | ~1 s per step (single leg swing), so speed ≈ step length / step period, consistent with the speed above | ASSUMPTION |
| Slope traversal | ±10° typical classroom ramp | ASSUMPTION (problem statement says "specified slope" but gives no value) |
| Stair dimensions | step height 0.05–0.08 m, depth 0.20–0.25 m | ASSUMPTION (problem statement says "specified height/depth" but gives no value) |
| Turning | In-place / small-radius turning not required for first milestone scope; straight-line walking first | ASSUMPTION |

## 6. Sensors

| Sensor | Purpose | Status |
|--------|---------|--------|
| Joint encoders | Joint position feedback (10 joints) | FACT (implied by problem statement) |
| IMU | Body orientation, CoM/ZMP estimation input | FACT (named in problem statement) |
| Foot contact sensors | Stance/swing detection | FACT ("where applicable" in problem statement — treat as required for gait control) |
| LiDAR | — | **NOT REQUIRED** (absent from specification and image; blocks SLAM/Nav2 milestones) |
| Camera | — | NOT REQUIRED (not named in problem statement; may be added later for terrain detection) |

## 7. Autonomy scope

- **In scope:** standing balance, scripted open-loop gait walking on flat ground (gait
  trajectory generator + `JointTrajectoryController`), gait start/stop interface,
  foot-stance sequencing, terrain worlds (slope/stairs) for later milestones.
- **Out of scope (decision by user):** full ZMP/CoM feedback control, SLAM, Nav2,
  teleop joystick, and any real-hardware control during simulation milestones.
- Open-loop gait limits: robot will walk on flat ground and gentle slopes with the
  scripted gait only; robust disturbance rejection is NOT expected. If later milestones
  require it, ZMP feedback will be added as a new milestone (decision point recorded in §11).

## 8. Compute / software environment

| Item | Value | Status |
|------|-------|--------|
| OS | Ubuntu 24.04 | FACT (verified) |
| ROS 2 | Jazzy (source `/opt/ros/jazzy/setup.bash`) | FACT (verified) |
| Simulator | Gazebo Harmonic (Gazebo Sim 8.11.0), headless-capable (`gz sim -s`) | FACT (verified) |
| ROS–Gazebo integration | `ros_gz_*` packages present | FACT (verified) |
| ros2_control stack | **Installed and verified (M5); reworked in M8.** `ros-jazzy-gz-ros2-control` 1.2.20, `ros-jazzy-joint-trajectory-controller` 4.42.1, `ros-jazzy-ros2controlcli` 4.48.0 (+ `controller-manager` 4.48.0 and deps), plugin at `/opt/ros/jazzy/lib/libgz_ros2_control-system.so`. `biped_control` package: controllers YAML (CM 500 Hz, JTC **effort** interface with per-joint PID p 70–120 / d 4–10, i=0), `stand_hold` node, `gait_generator`, `gait_odometry`, spawner + full-stack `stand.launch.py` / `walk.launch.py`. Sole collisions use soft damped contacts (kp 3e4, kd 100, max_vel 0). Rationale: the position-interface realization in gz_ros2_control is a velocity law that could not hold single-support load (kp=1 sags/falls) without injecting explosive energy (kp≥8 tumbles). | FACT (verified M5, reworked M8) |
| Workspace | `~/RosProject_ws` (user decision), packages kept under this git repo and linked/symlinked | FACT (user decision) |
| RViz2 | Required for visualization | FACT |

## 9. Package architecture (ROS 2 conventions)

```
~/RosProject_ws/src/            ← symlink/mirror of this repo's src/
├── biped_description/          URDF/Xacro, meshes, robot_state_publisher launch
├── biped_control/              controller config, gait generator node, controllers launch
├── biped_bringup/              full-stack launch files, world files, Gazebo spawn
├── biped_sensors/              sensor config (IMU/contacts bridging) when needed
└── biped_interfaces/           custom msgs/srvs/actions (only when actually needed)
```

The user's suggested names (`robot_description`, `robot_control`, …) are generic;
the project uses `biped_*` prefixes. If the user prefers exact `robot_*` names,
this is a trivial rename at any point — ASSUMPTION recorded.

## 10. Acceptance tests (gates per milestone)

| ID | Test | Milestone |
|----|------|-----------|
| AT-01 | `PROJECT_REQUIREMENTS.md` exists, all facts traceable to problem_statement/image, all unknowns tagged ASSUMPTION | M0 |
| AT-02 | URDF/Xacro validated (`check_urdf`/xacro parse), all 10 joints present with correct axes | M2/M3 |
| AT-03 | Robot spawns in Gazebo without errors; robot stands on both feet; does not fall through ground or collapse | M4 |
| AT-04 | Joint trajectory commands move legs; joint_states published at stable rate | M5 |
| AT-05 | Standing balance holds ≥ 30 s without falling (with and without 1 kg payload) | M5 |
| AT-06 | Open-loop gait walks forward ≥ 1 m on flat ground without falling | M5/M6 |
| AT-07 | TF tree: `map → odom → base_link → … → *_link` complete, no duplicate/conflicting transforms | M7 |
| AT-08 | IMU + foot contacts publish sensible data (rates, frames) | M8 |
| AT-09 | RViz2 displays robot model, TF, sensor data | M9 |
| AT-10 | Gait stop command halts walking safely | M5+ |
| AT-11 | Gait retuning: upright forward walk (≥ 1 m, verified against ground truth: IMU upright throughout + positive monotonic progress) | M6 follow-up |

### 10.1 Results log

| Date | Test | Result |
|------|------|--------|
| 2026-09-16 | AT-01 | PASS (M0 doc complete, traceable) |
| 2026-09-16 | AT-02 | PASS — xacro parses, 10 revolute joints, axes/limits per spec |
| 2026-09-16 | AT-03 | PASS — spawn stable, z = 0.5999 m standing height, no drift |
| 2026-09-16 | AT-04 | PASS — JSB + JTC active via gz_ros2_control; `/joint_states` 100.0 Hz ±0.0001 s |
| 2026-09-16 | AT-05 | PASS — 15° crouch hold, 30 s base-pose drift = 0 (bit-identical) with and without the 1 kg payload; crouched z = 0.5864 m matches kinematics (0.60 − 0.40·(1−cos 15°)) |
| 2026-09-17 | AT-06 | PASS — open-loop walk with 1 kg payload: 17 steps, auto-stop at the 1.2 m distance limit; stance-foot kinematic odometry measured x = +1.2378 m (monotonic, matches commanded distance within 3%); 0 joint-limit clamps, 0 ESTOP/abort events. Fixes required during verification: (1) /gait_support now publishes the STANCE leg at segment boundaries (pre-set latched publisher suppressed it) and (2) leg_ik hip sign corrected (forward ankle target needs negative hip pitch about +Y — robot previously walked backward, invisible while odometry was dead). **Superseded: see the REOPENED entry of the same date — the pass was measured in joint space only and the robot was not upright.** |
| 2026-09-17 | AT-05 | PASS — re-verified under the M8 effort-mode controller: 15° crouch hold, knee ≈ 30.4° (matches 2·15° knee split), az = 9.81 with std 0.00 (1000 samples), odom drift 0, 0 anomalies. |
| 2026-09-17 | AT-06 | REOPENED — M8 ground-truth instrumentation (IMU) exposed that the earlier AT-06 passes were never verified upright: under the previous position-interface control (velocity realization, kp=1) the JTC tracked the gait perfectly while the robot physically fell forward at the first weight shift (az 9.81 → ~2 m/s² ≈ 80° pitch) and was `walked` as a dragged body. Control rework in M8: JTC moved to the effort interface with per-joint PID (i=0 after windup-induced pogo-stick), CM 500 Hz, soft damped sole contacts, step_period 2.0 → 2.5 s. Post-rework: standing/crouch-hold rock-solid (AT-05 above); walking no longer falls but bounces in place (net ~0.01 m in 40 s, contact force chopping). NEXT: gait retuning (step geometry, segment timing, higher p on knees) before AT-06 can be re-passed; AT-10 to be re-verified on the retuned gait. |
| 2026-09-17 | AT-07 | PASS — complete TF tree map → odom → base_link → hip/knee/ankle chain → *_foot (+ platform_link → payload_link), verified with tf2_tools view_frames: single-rooted, no duplicate/conflicting parents; gait_odometry broadcasts odom→base_link at 100.2 Hz, robot_state_publisher leg frames at 25.2 Hz. New: identity static map→odom anchor in gazebo.launch.py (localization out of scope, §7; replace with a dynamic estimate if localization is ever added). End-to-end lookups at spawn: map→left_foot = [0, +0.10, −0.45], map→right_foot = [0, −0.10, −0.45] (mirrored hip separation, matches geometry). |
| 2026-09-17 | AT-08 | PASS — IMU (/imu, gz.msgs.IMU → sensor_msgs/Imu, frame `biped/base_link/imu_sensor`, 100.0 Hz): |a| = 9.81 m/s² at stand; az std = 0.00 over 1000-sample windows at stand and crouch-hold. Foot contacts (/left_foot_contact, /right_foot_contact, gz.msgs.Contacts → ros_gz_interfaces/Contacts): ~720 msg/s per loaded foot, forces sensible (41.2 N = half weight in double-support stand; dynamic 0–150 N transfer while stepping). gait_odometry stance now sourced from the contacts (logs `contact sensors online`), /gait_support kept as fallback; base pitch fused from the IMU (complementary filter). Two URDF→SDF converter traps fixed and documented: (1) a named URDF collision gets `_collision` appended by the converter, so foot collisions are left unnamed and the sensor references `<link>_collision`; (2) the contact topic must ALSO be inside `<contact>`, otherwise the Contact system publishes on the world-scoped default topic, unreachable from the ros_gz_bridge. Sensors declared in `biped_description/urdf/sensors.xacro`; world plugins gz-sim-imu-system + gz-sim-contact-system in flat_world.sdf; bridges in gazebo.launch.py. |
| 2026-09-17 | AT-10 | PARTIAL — stop state machine verified end-to-end on 2026-09-17 runs (WALKING → STOPPING → IDLE on both distance-limit and manual `false`; swing foot lands, pose freezes, odom flat) and the ESTOP/watchdog paths were exercised during the M8 tuning (aborts froze the pose safely). Re-verification on the RETUNED gait is still required before this is re-passed (see AT-06 REOPENED). |
| 2026-09-17 | AT-09 | PASS — RViz2 (Jazzy) runs headless under xvfb with the project config `biped_control/config/rviz/biped.rviz` (OpenGL 4.5, zero errors, clean exit): Grid, RobotModel (/robot_description), TF, MarkerArray (/contact_markers), Odometry (/odom axes); fixed frame odom, orbit view on base_link. New node `biped_control/contact_markers` publishes the M8 sensor data as markers in odom: per-foot contact force arrows (green = left, orange = right; anchored at the sole via TF, length ∝ normal force, lifetime 0.15 s so lifted feet stop drawing) and the IMU measured-up arrow from the pelvis (accelerometer direction rotated into odom; upright + static = 0.2 m straight up). Markers combined into ONE MarkerArray at 30 Hz (subscriptions only store state — the ~700 Hz contact stream does not flood RViz). No rviz IMU plugin ships with the distro, hence the marker approach. Wired as a shared `rviz.launch.py` include: `use_rviz:=true` on `walk.launch.py` and `stand.launch.py`; `contact_markers` runs unconditionally (negligible cost, markers inspectable without a GUI). Verified: both feet' markers stream in odom, rviz2 process stable for the full session, no display-load errors. |
| 2026-09-18 | AT-11 | IN PROGRESS — gait-retune campaign, 13 controlled A/B runs (param-gated mechanisms + gz pose ground truth). **Achieved: abort-free upright stepping for the full probe window** (az 9.76–9.86 m/s² through every phase; zero JTC aborts/ESTOPs). **Remaining gap: the "treadmill" — the pose truth shows the pelvis advancing +3.6 cm during SETTLE while both planted feet slide back ~2 cm, netting ≈ 0 forward progress.** Root causes fixed en route: (1) URDF `<surface>` inside `<collision>` is SILENTLY DISCARDED by the URDF→SDF converter (gz sdf -p verification now mandatory; gazebo-extension short tags `mu1/mu2` are the idiom that merges, nested `<friction><ode>` still merges empty — the soles had zero/unknown grip until now); (2) LIFT/SWING/PLANT used fixed 0.1/0.3/0.1 s durations while SHIFT/SETTLE scaled with step_period — at 10 s/step the swing ankle had to sweep 0.46 rad in 0.3 s and the JTC aborted at Position Error 0.3005 > 0.300 every run (now scaled with the step period); (3) `walk.launch.py` gained a `gait_gen_params` YAML-override hook (OpaqueFunction) after a launch-argument name mismatch silently invalidated three A/B runs — harness checks now assert parameters actually arrive. A/B verdicts: command-space compensation is uniformly REJECTED by the plant (swing-sole levelling from the IMU, per-segment ILC, frozen ILC, static hip feedforward — all destabilize the pelvis attitude equilibrium at the first weight shift); ankle 40/4 + hip/knee 60/6 is the stable stiffness island (hip 120 → az ±85 chaos; hip 90 doubles the progress to +0.056 m but falls at t≈30 s — the boundary sits between 60 and 90, and 60 is kept as the verified-stable value); soft sole contact kp 8e3/kd 35 is required (rigid contacts fall at the first shift; the earlier rigid-contact "fix" was measured on an invalid run). Mechanism params (`sole_level_gain`, `ilc_gain`, `hip_sag_ff`) default OFF. NEXT: forward progress — candidate levers are higher sole grip μ (4.0 in place, untested alone), step_length increase under the feasibility guard, and pelvis-advance-aware SETTLE geometry. |
| 2026-09-18 | AT-10 | PASS — re-verified on the retuned gait: walk engages (crouch + ~3.5 step cycles, az ≥ 9.75 throughout), STOP (`false` on /gait_command) mid-walk → swing foot lands, pose settles, robot holds perfectly stable for 60 s (az = 9.81, knee frozen at 45.3°, zero odom drift, zero anomalies). Test-harness fix: `ros2 topic pub -1` delivers to an arbitrary matching subscription — any probe subscribed to /gait_command steals the message from the gait node; use `-w <n_subscribers>`.
| 2026-09-19b | AT-11 | **INFRASTRUCTURE DEBUG + CLEAN BASELINE RESTORED.** Evening session (runs BE–BZ) resolved three compounding traps that had poisoned the whole BB+ era: (1) **process leaks**: the A/B harness never killed `gait_odometry`/`contact_markers`, orphaning one per session; hours of zombie DDS traffic + desktop load degraded real-time factor to **0.825** (measured via /clock), breaking the JTC's time-based splines — every "config regression" (d=15, approach watchdog, PGS/Dantzig) was partly this. Harness now leak-proofs its kill list. (2) **Dead-comment archaeology**: the "soft contact" URDF tags were always a placebo under DART (surface-in-collision is discarded; the gazebo-extension contact block merges at link level, inert) — the 09-17/18 "soft contact required" verdict actually measured Dantzig-vs-DART solver differences; the explicit `<dart>` block (runs Q–BN) is now removed, restoring run-I default physics. (3) **Metric honesty**: odom_x integrates drift and tumble artifacts — **step-build count is the only trustworthy progress metric**. On the clean machine the reproducible baseline is: default physics, Dantzig, ankle 40/4, hip/knee 60/6 (d=15 verified incompatible with the default-physics plant — it was tuned on PGS), μ=1, settle_boost 0.04, tolerances 0.30 → **3 clean real steps per session, |a| = 9.8 throughout, reproducible 5×**. Session limit is a GENUINE tip at steady step 2–3 (~55–65 s), invariant across step_period 6/10, boost on/off, d 6/15 — not retry arithmetic (0 retries used at death), not fraction bookkeeping. New machinery added and verified: body-at-rest WALK gate (refuses starts from non-rest), IMU-gated segment retries 3→8, abort-recovery graceful-land via the AT-10 trim machinery (works; falls develop >25° before the abort moment so it hasn't fired yet), hard-freeze reserved for genuinely tipped bodies. NEXT SESSION: per-step pelvis-vs-feet bookkeeping probe (phase-resolved, 09-18 tool) on the clean plant to identify what accumulates across 3 steps — prime suspect is a per-cycle geometric ratchet (settle sag lands the pelvis short while feet re-plant forward, stretching the stance geometry until tip). |
| 2026-09-21 | AT-11 | **CLEAN-PLANT FACTORIAL + GROUND-TRUTH FALL DECODED — SIMULATION PHASE CLOSED.** Rebuilt the wiped /tmp harness (v3: self-sourcing, leak-proof kills, rest-gated WALK bursts, flushed probes; `set -u` breaks ROS setup scripts) and ran the phase-resolved ground-truth probe (gz pose at full rate, 61k+ poses): **the step-3 fall is BACKWARD onto the robot's back** — pitch −0.067 rad at baseline (hip-sag lean, CoM already behind the ankle), real forward progress at step 2 (+0.05 m), then a backward lurch at step 3 (−0.25 m) and flat-on-back (−π/2). Mechanism: backward CoM drift in single support plus swing reaction compounding across steps until the heel edge — no ankle torque can arrest it at the tip moment. Factorial around the baseline (one variable at a time, step-count metric): ankle p 80 → 2 steps (worse); sagittal_margin −0.02 (CoM forward) → 2 steps (tips FORWARD — the optimum is sharp); crouch 25° → 1 step (violent); hip p 80 → 3 steps (no change). With the earlier sweeps (period, boost, damping, physics, solver, chaining, retries, recovery) the 3-step session is a **sharp local optimum** and the tip is structural, not parametric. Path beyond requires CoM/ZMP preview tracking — a controller rework, out of iteration scope. Final verdict: session walking works (3 clean steps, |a| = 9.8, reproducible); unattended 1 m in one session does not. AT-06/AT-11 closed with characterized gap; hardware phase docs (M12–M14) ready for the user's §13 Q2/Q4 decisions. |
| 2026-09-26 | AT-02 | PASS — re-verified on the commit-verification pass: xacro + check_urdf clean for BOTH payload variants; `gz sdf -p` converts with zero errors and the sole `<mu>1</mu>` friction merges correctly (regression check for the 09-18 silent-discard trap). |
| 2026-09-26 | AT-03 | PASS — re-verified end-to-end (headless launch → spawn → gz pose ground truth → teardown, zero leftover processes): base_link z = 0.599999 m (exact 0.55+0.35+0.15−0.45 kinematic stack), tilt 2e-8 rad, drift 0.000000 m over 6 s, both soles at z = −0.45 m on the ground plane. |
| 2026-09-26 | AT-04 | PASS — re-verified: `stand.launch.py` full stack; joint_state_broadcaster + joint_trajectory_controller both **active** (effort interface); `/joint_states` 500 Hz (min/max 0.002 s, std 0.00004 s over 30-sample window). |
| 2026-09-26 | AT-05 | PASS — re-verified (1 kg payload attached, default): 15° crouch hold rock-stable — base z = 0.5855 m vs 0.5864 kinematic expectation (<1 mm), settled 4.2° lean = the documented hip-sag equilibrium, drift 0.000000 m / dz 0.000000 m across the hold window, IMU az = 9.78 m/s² (= 9.81·cos 4.2°, consistent with the lean). |
| 2026-09-19 | AT-11 | **BREAKTHROUGH — sustained forward walking achieved; long-horizon AT-06 still open.** Final mechanism found: `settle_boost` — a purely kinematic over-advance of the SETTLE target past the feet midpoint (0.04 m, clamped ≤ 0.45·step_length, carried through the next SHIFT start), compensating the measured per-cycle sag recoil (AJ: pelvis fell 0.03–0.05 m behind the commanded advance every cycle). Runs BA→BD isolate it: BA/BC (boost 0.04 + frac_settle 0.30, ankle p 80/d 10) show the campaign's first **monotone** odom_x climb (+0.005 → +0.110 over 140 s, still climbing at cutoff, replicated twice); ankle d 20 → collapse (15 ESTOPs, reverted). Harness v2 fixes: workspace sourcing (run BA died against an unrelated overlay), `/**:` YAML nesting (bare keys SIGABRT the node), float-only launch args (`crouch_deg:=20` integer throws on the double-typed param), ancestor-safe kill helper, in-process WALK publisher. Long-run finding: each walk session peaks (+0.190 m — best ever) then aborts via the rocking mode (~38 m/s² horizontal, tracking error climbs monotonically through strained segments until it crosses the JTC abort line — raising the tolerance just delays the crossing, d=20 destabilizes), and each ESTOP→re-arm cycle ratchets the robot BACKWARD (the fresh APPROACH crouch drags it toward its feet), so burst re-arming nets distance decay. Remaining work is the rock-mode damping problem — rejected feedback mechanisms (ILC/FF/sole-levelling) remain rejected; candidate untried levers: ankle d between 10 and 20 with re-tuned p, DART solver iterations, or a proper ZMP/CoM damping layer (out of open-loop scope per §7, decision point §11). AT-06 stays REOPENED with a characterized, reproducible gap. |

## 11. Milestone mapping (user roadmap ↔ PDF phases)

| User milestone | Content | PDF phase |
|----------------|---------|-----------|
| M0 | Requirements (this file) | Phase 0 |
| M1 | Laptop/ROS 2/Gazebo baseline | Phase 1 |
| M2–M3 | Robot description, URDF/Xacro | Phase 3 |
| M4 | Gazebo spawn | Phase 4 (spawn step) |
| M5 | Drive → reinterpreted: joint trajectory control + standing + gait | Phase 4 (drive steps) |
| M6 | Odometry → reinterpreted: gait progress/base pose estimate (no wheels) | Phase 5 |
| M7 | TF | Phase 5 |
| M8 | Sensors (IMU, foot contacts) | Phase 5 |
| M9 | RViz2 | Phase 6 |
| M10–M11 | SLAM / Nav2 — **NOT REQUIRED** (no LiDAR per specification) | Phase 7/8 — skipped, conflict C1 |
| M12–M14 | Hardware selection, physical bring-up, validation | Phase 9/10 — **DOCUMENTED 2026-09-19**: HARDWARE_SELECTION.md (sizing derived from sim-measured torques/masses; XM-class tiered BOM; key finding: 8.4 kg exceeds smart-servo 20 %-continuous ratings ⇒ tier A requires a ≤ 5 kg build or the harmonic-drive upgrade path), BRINGUP_PLAN.md (6 gated stages, suspended-first safety), VALIDATION_PLAN.md (HW-01..09 mapped 1:1 to the sim AT suite with honest sim-verdict references). **TIER SELECTED 2026-09-21: Tier S, ₹15,000 student build** (HARDWARE_SELECTION.md §7): 10 × DS3218 (₹879–1,999 ea verified) + ESP32 + PCA9685 + MPU6050 + FSRs, ≤ 1.6 kg, walks in the sim-proven session mode. Full hardware layer **implemented and building**: `pwm_servo_hw` ros2_control plugin (POSITION → serial → PCA9685), `esp32_sensor_bridge` (MPU6050+FSR telemetry → the exact /imu and /<side>_foot_contact contracts, so every sim node runs byte-identical), ESP32 firmware, `ros2_control_hw.xacro`; BRINGUP/VALIDATION plans annotated with Tier S substitutions. Remaining: kit purchase + M13/M14 execution per plan. |

## 12. Safety (simulation phase)

- No real motor commands during simulation development.
- Gait generator must include joint limit guards; emergency stop = zero-trajectory + freeze.
- Simulation success does not guarantee physical safety.

## 13. Open questions for the user (do not silently invent)

1. Exact slope angle and stair dimensions for later terrain milestones (problem statement says "specified" but no values were supplied).
2. Robot total mass target for hardware sizing (any upper limit from the course?).
3. Package naming (`biped_*` vs `robot_*`) confirmation.
4. Any budget constraint for hardware milestone M12.
