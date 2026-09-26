# VALIDATION_PLAN.md — M14 (Validation, Phase 10)

Hardware acceptance suite. Each test maps 1:1 to a simulation acceptance test
(PROJECT_REQUIREMENTS.md §10 + §15 log) so results are directly comparable; the sim
verdict column states what the same gate achieved in Gazebo. Precondition: all
BRINGUP_PLAN.md gates pass.

## Environment & instrumentation

- **Ground truth** (replaces gz pose): video camera perpendicular to the walk lane
  with floor tape every 0.1 m, OR taped lane + operator counting steps (step length is
  commanded 0.06 m; ground-truth distance = tape marks at start/end of each session).
- **Attitude**: the onboard BNO085 publishes /imu exactly as the sim IMU did — the
  az = 9.81 ± wobble criterion carries over verbatim. **Tier S (selected
  2026-09-21): MPU6050 via `esp32_sensor_bridge` at ~50 Hz publishes the same
  /imu contract; widen the az band to ±0.3 and expect noisier tilt (documented
  sensor substitution, HARDWARE_SELECTION.md §7).**
- **Foot contacts**: FSR pair per foot → adapter node publishes the same
  `/left_foot_contact` / `/right_foot_contact` semantics (normal force).
- **Session log**: `ros2 bag record /imu /odom /joint_states /gait_status
  /left_foot_contact /right_foot_contact` per session, plus a timestamped phone video.

## Acceptance tests (hardware)

| AT | Gate | Pass criterion | Sim verdict (reference) |
|---|---|---|---|
| HW-01 (AT-01/02/03 analog) | Description/TF/JTF states | Full TF tree from base_link to both soles; `/joint_states` tracks hand-moved joints ±0.01 rad. **Tier S: no servo readback exists — replaced by a commanded sweep (`S` pulses through each joint's range) with visual verification and the commanded-echo /joint_states.** | PASS in sim (AT-07 tree verified) |
| HW-02 (AT-05) | Stand hold | 15° crouch hold 60 s, az = 9.81 ± 0.1, knee sag within 2× sim's 0.13 rad, temp rise < 15 °C | PASS (az std 0.00 over 1000 samples) |
| HW-03 (AT-08) | Sensors | /imu 100 Hz; FSR reads ~½ weight per foot in double support; force transfers visible during stepping | PASS (41.2 N per foot, 0–150 N dynamic) |
| HW-04 (AT-09) | Visualization | RViz2 on the operator laptop shows model, TF, contact arrows, IMU up-arrow live from the robot | PASS (headless-verified stack) |
| HW-05 (AT-11) | Gait retune on hardware | ≥ 3 consecutive walk sessions with net positive distance, zero falls (boom catches excepted); the AT-11 config is the starting point, stiffness island re-hunted per BRINGUP stage 3 | PARTIAL in sim (stable upright stepping; session walking demonstrated) |
| HW-06 (AT-06) | **Walk 1 m** | Robot walks ≥ 1.0 m **cumulative across sessions** against tape/camera ground truth, upright throughout (az ≥ 6 m/s² excepting bounded wobble), zero falls | NOT MET in sim (best session +0.19 m, monotone; long-horizon gap characterized) |
| HW-07 (AT-10) | Safe stop | STOP (`false` on /gait_command) mid-walk → swing foot lands, pose settles, robot holds stable 60 s with zero odom drift; physical E-stop cuts motors within one control cycle | PASS in sim (60 s hold, zero drift) |
| HW-08 | Payload walk | Repeat HW-06 with the 1 kg payload installed (platform mounted) | sim ran with payload from M2 onward |
| HW-09 | Slope (if terrain values supplied — §13 Q1 still open) | Gentle-slope session walk per the spec's "specified slope" | out of sim scope (flat worlds only) |

## Protocol

1. Each test logged in PROJECT_REQUIREMENTS.md §15 with date, config hash
   (`git rev-parse HEAD` + params file), bag filename, and raw numbers — same
   discipline as the sim campaign (no verdict without a measurement).
2. A fall during any test = test abort, boom/spotter catch, 30 s settle, log, and
   only then retry (max 3 retries per test per day — thermal and gear-wear limit).
3. Any deviation from the sim-transferred config is logged as a hypothesis with its
   A/B result, keeping the sim↔hardware comparison honest.

## Known gaps carried from simulation (honest status)

- **Long-horizon forward progress** (HW-06): the sim's best is +0.19 m per session
  with a backward ratchet on re-arm cycles. If the hardware reproduces this, the
  1 m gate is met by session accumulation only if the ratchet is solved (candidate
  levers documented in the 2026-09-19 AT-11 log entry).
- **Rock-mode damping**: the abort-every-10–20-steps mode will likely appear on
  hardware with different constants; the IMU-gated re-arm and session logic transfer,
  the numbers do not.
- Slope/stair terrain (PDF phases beyond the flat-gait scope) remains unscheduled
  pending §13 Q1.
