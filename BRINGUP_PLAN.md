# BRINGUP_PLAN.md — M13 (Physical bring-up, Phase 9/10)

Precondition: HARDWARE_SELECTION.md tier chosen and kit received. This plan brings the
real robot from bags-of-parts to "standing, then walking in sessions" using the
simulation stack **unchanged except for the hardware-interface plugin and sensor
adapter** (§5 of HARDWARE_SELECTION.md).

> SAFETY (§12 applies verbatim): bench-test with the robot SUSPENDED (both feet off
> the ground) until every gate below passes. The physical E-stop cuts the motor rail;
> the software ESTOP (zero-trajectory + freeze, §12) stays armed at all times. Never
> stand the robot on a table edge; use a test boom or overhead gantry line for first
> stand attempts.

## ★ SELECTED TIER (2026-09-21): Tier S — ₹15,000 student build

This plan's stages apply with the Tier S substitutions below (HARDWARE_SELECTION.md
§7); the Dynamixel-specific steps in Stages 0–1 collapse into one ESP32/PCA9685
bench:

| Plan element | Tier S implementation |
|---|---|
| Compute | Dev PC runs the full ROS stack (identical to sim); ESP32 does realtime I/O only. An RPi can replace the PC later for untethered demos. |
| Servo bus (Stage 0) | ESP32 + PCA9685 + 10 × DS3218; flash `biped_control/firmware/esp32_firmware.ino`; bench-verify each channel with the `S<ch>,<us>` serial command (500–2500 µs sweep), record per-joint neutral pulse and direction sign into `ros2_control_hw.xacro` |
| Hardware interface | `biped_control/PwmServoHardware` plugin (`pwm_servo_hw.cpp`): POSITION commands → serial; no readback, no effort — documented losses |
| Sensors (Stage 3) | MPU6050 + 4 × FSR-402 on the ESP32; `esp32_sensor_bridge` republishes the exact `/imu` + `/left_foot_contact` + `/right_foot_contact` topics — **all sim nodes byte-identical** |
| Control mode | JTC runs in POSITION mode (controllers.yaml `p: 1.0`); the sim's effort-mode PID has no PWM-servo analog |
| E-stop | Physical toggle on the 6 V servo rail (logic stays up) — §12 verbatim |
| Gate 0 equivalent | `S` commands move every servo; `T` telemetry streams at 50 Hz; `ros2 control list_hardware_interfaces` shows 10 × position claimable |

## Stage 0 — Bench bring-up (no legs assembled)

1. Flash Raspberry Pi 5 with Ubuntu 24.04 + ROS 2 Jazzy; install the workspace
   (`~/RosProject_ws`, same layout as sim).
2. Wire U2D2 + power hub to ONE XM540; verify `dynamixel_sdk` ping, ID, firmware,
   baud (4.5 M), and torque-enable/disable cycle.
3. Extend to the full daisy chain; assign IDs 1–10 to match the sim joint order
   (`left_hip_roll, left_hip_pitch, left_knee, left_ankle_pitch, left_ankle_roll,
   right_…` — gait_generator's `joints_` vector is the contract).
4. Verify every servo: position read-back accuracy (<0.5°), temperature limit config
   (default 72 °C), max current limit (protects the 20 %-continuous rule), return
   delay time.

**Gate 0:** `ros2 control list_hardware_interfaces` (via the new `dynamixel_hw`
plugin) shows 10 × {position, velocity, effort} claimable; pinging all 10 IDs
simultaneously at 100 Hz shows no timeouts.

## Stage 1 — `dynamixel_hw` ros2_control plugin (the only new code)

1. Implement `hardware_interface::SystemInterface`:
   - `export_state_interfaces`: position/velocity/effort per joint.
   - `export_command_interfaces`: effort (mapped to Dynamixel current mode, like the
     sim's JTC-effort architecture) OR position (Dynamixel internal loop) — START in
     position mode (matches the sim's command stream 1:1), keep effort as the
     documented fallback.
   - `read()`: bulk-read positions at 100 Hz min; `write()`: goal-position write.
2. Reuse `biped_description` URDF verbatim — update only the link masses to the real
   build and the `joint_eff_lim` to the servo's max current torque.
3. Create `controllers_hardware.launch.py`: controllers.launch.py minus gazebo, plus
   `dynamixel_hw`.

**Gate 1:** all 10 controllers spawn and `ros2 topic echo /joint_states` matches hand
moved joints at ±0.01 rad.

## Stage 2 — Suspended stand (AT-05 analog)

1. Robot hanging in the rig; run `stand.launch.py` hardware variant.
2. `stand_hold` commands the AT-05-verified 15° crouch — servos reach it, hold
   60 s, temperature rise < 15 °C per joint.
3. Repeat with the robot's feet JUST touching (weight-shifting begins) — watch the
   M8-equivalent foot FSR readings appear and match ~½ weight per foot.

**Gate 2:** crouch hold 60 s, zero tracking anomalies, FSR pair reads total weight ±10 %.

## Stage 3 — Floor stand (first unsupported)

1. Robot standing on the floor in the boom rig (slack line, catch-only).
2. Engage stand-hold; verify the sim-predicted sag is present (0.10–0.15 rad knee) —
   record the REAL stiffness island starting point from it (compare k_p sweep on one
   joint at a time).
3. E-stop drill: physical cut mid-hold → servos release → robot caught by boom.
   Repeat 3×.

**Gate 3:** free-standing hold ≥ 120 s; E-stop drill passes; recorded sag
within 2× of the sim's 0.13 rad (else STOP and re-derive §1.2 of
HARDWARE_SELECTION.md with the real numbers).

## Stage 4 — First walk sessions

1. Reproduce AT-11's final config through the real stack: step 0.06 m, period 10 s,
   crouch 20°, settle_boost 0.04, frac_settle 0.30, ankle p 80 / d 10, hip/knee p 60.
2. Sessions: WALK burst → auto-settle at first tracking anomaly (the node's IMU gate
   + JTC abort do this unchanged) → 30 s settle → re-arm burst. Distance per session
   recorded with tape/camera ground truth.
3. Expect 0.05–0.10 m per session initially (sim predicted session banking).

**Gate 4:** ≥ 3 consecutive sessions with net positive distance and zero falls
(boom catches excepted). Then the AT-06 hardware attempt (VALIDATION_PLAN.md).

## Stage 5 — Loose the boom (only after Gate 4)

Soft-rope spotter, operator hand on E-stop, 1 m walk attempt. Full AT suite per
VALIDATION_PLAN.md.
