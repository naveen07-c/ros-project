# HARDWARE_SELECTION.md — M12 (Hardware selection, Phase 9)

All sizing numbers below are **derived from the simulation campaign** (AT-05 → AT-11,
see PROJECT_REQUIREMENTS.md §15), not invented: masses are the URDF assumption values,
torques are the moments measured through the sim control loop (documented sag 0.13 rad
at known stiffness ⇒ torque = k_p × error), and contact loads are the M8 sensor streams.

> STATUS: selection document only — nothing purchased. Budget question (§13 Q4) and
> mass target (§13 Q2) remain open with the user; three tiers are provided so the
> decision can be made without rework.

---

## 1. Sizing basis (measured in simulation)

### 1.1 Mass budget (URDF assumption values, PROJECT_REQUIREMENTS §3/§4)

| Element | Mass |
|---|---|
| Pelvis 1.2 + platform 0.8 + payload 1.0 | 3.0 kg |
| Per leg: thigh 0.55+0.40 (knee act) + shin 0.45+0.30 (ankle act) + foot 0.30 | 2.00 kg |
| Hip actuators (2 × 0.40, mounted at pelvis) | 0.80 kg |
| **Total (matches M8-measured 82 N contact load)** | **8.4 kg** |

### 1.2 Joint torque demands (measured through the effort-interface control loop)

| Joint | Continuous requirement (evidence) | Peak (2× dynamic margin) |
|---|---|---|
| Hip pitch | ~15 N·m (gait_generator gravity moment, 0.13 rad sag at k=60) | 30 N·m |
| Knee | ~8 N·m (stance knee SS sag, code-documented 8/60 = 0.133 rad) | 16 N·m |
| Ankle pitch | ~6.4 N·m (inverted-pendulum gradient m·g·h = 49 N·m/rad × 0.13 rad) | 13 N·m |
| Hip/ankle roll | 2.5–5 N·m (3–6 cm CoM offset in single support) | 10 N·m |
| *Design check* | *URDF effort limit 40 N·m is consistent with all of the above* | |

**Design rule applied:** smart serial servos are rated by *stall* torque but may only
hold **~20 % of stall continuously** (Robotis published figure for X-series). The
continuous column above is therefore the binding constraint, not stall.

### 1.3 The key finding from AT-11 (directly transferable to hardware)

- The sim robot is **mass-optimal at the edge of the XM-class actuator envelope**: at
  8.4 kg the stance-knee/hip continuous torques exceed every smart-servo's 20 %
  continuous rating (XM540: 10.6 N·m stall / 2.12 N·m continuous).
- The stable operating mode found in AT-11 — **slow quasi-static steps (10 s/step),
  session walking with graceful stop/settle between sessions** — is exactly how a
  duty-cycled servo build must walk. The sim's session-walking result is therefore a
  *prediction*: the real robot will walk in sessions, banking distance per session.
- The soft-sole-contact and stiffness-island findings (hip/knee p 60, ankle p 80)
  are control-side and will be re-tuned on hardware; the geometry and session logic
  (settle_boost, sag-aware tolerances, IMU-gated re-arm) transfer as-is.

---

## 2. Actuator selection

| Tier | Hips + knees (6×) | Ankles (4×) | 10× actuator cost | Real-mass consequence |
|---|---|---|---|---|
| **A — XM-class (recommended baseline)** | XM540-W270-T: 10.6 N·m stall, 2.12 cont. ($482–494 ea, robotis.us 2026-09) | XM540-W270-T (ankle pitch gradient is the largest term) | ~$4,870 | Robot must be built ≤ 5 kg total (payload 0.5 kg) ⇒ continuous needs scale to ≤ 1.6 N·m/knee — inside 20 % rating with margin |
| **B — Mixed** | XM540-W270-T ×6 (~$2,900) | XM430-W210-T ×4: 4.1 N·m stall, 0.82 cont. ($310 ea) | ~$4,140 | Ankle margin thin at 5 kg (gradient scales with mass ⇒ 6.4 → 3.8 N·m vs 0.82 continuous) — viable only with a lightened pelvis; NOT recommended |
| **C — Desktop demonstrator** | XL430-W250-T ×10: 1.5 N·m stall ($44.90 ea) | XL430-W250-T | ~$450 | Only a ≤ 1.5 kg desktop biped (payload 0.2 kg); walks quasi-static sessions, demonstrates the full control stack on real hardware |

Notes:
- Prices verified 2026-09 (robotis.us, robotshop); U2D2 interface ~$43 + power hub ~$30.
- Higher-torque alternatives (MyActuator RMD-X / harmonic-drive brushless) would remove
  the mass constraint but at >$150/joint and with a heavier integration burden
  (no built-in position loop at the bus level, custom driver node required). Listed as
  the upgrade path if Tier A's 5 kg budget cannot hold the payload requirement.
- No potentiometer-type hobby servos: the sim architecture (effort interface, per-joint
  PID at 500 Hz, abort-on-tracking-error) presumes position feedback + torque limiting,
  which only smart serial servos provide at this price point.

## 3. Sensors (matching the M8 sensor suite)

| Sim sensor | Hardware pick | Interface | Cost |
|---|---|---|---|
| Pelvis IMU (/imu, 100 Hz) | Bosch BNO085 (Adafruit breakouts verified available; 6/9-DoF fusion on-chip) | I²C/UART → RPi | ~$25 |
| Foot contacts (/left_foot_contact, /right_foot_contact) | 2× FSR-402 per sole (heel + toe), matches the sim's per-foot normal-force model | ADC (ADS1115) | ~$10 + $14 |
| Ground truth for validation | Vicon/AR-marker session OR tape-measure + camera — AT-06 requires ground-truth distance, sim used gz pose | — | $0–borrowed |

## 4. Compute & power

| Item | Pick | Rationale | Cost |
|---|---|---|---|
| Onboard computer | Raspberry Pi 5 (8 GB) | Ubuntu 24.04 + ROS 2 Jazzy = identical stack to sim; runs gait_generator + gait_odometry unchanged | ~$80 |
| Servo bus | U2D2 + U2D2 power hub | Jazzy-supported dynamixel_sdk; the JTC-effort architecture maps to a thin ros2_control hardware_interface | ~$75 |
| Battery | 4S LiPo 2200 mAh (14.8 V) | XM540 rated 11.1–14.8 V; 5.5 A stall × 1 active leg + logic ≈ 15 min session budget | ~$30 |
| E-stop | Physical toggle cutting motor rail only | Matches §12: logic keeps power, servos release (freezewithout-torque = the sim's ESTOP analog) | ~$5 |

## 5. Sim → hardware mapping (what ports, what changes)

| Sim component | Hardware counterpart | Change required |
|---|---|---|
| `gait_generator` (action client → JTC) | Same node → JTC over new `dynamixel_hw` ros2_control interface | Hardware interface plugin only (position+velocity+effort claim); node untouched |
| `gait_odometry` (contacts + IMU fused) | Same node; `/left_foot_contact` adapter maps FSR ADC → Contacts-like msg | Small adapter node (~100 LOC) |
| `stand_hold`, `contact_markers`, RViz2 | Unchanged | — |
| gz physics / soft contacts | Real rubber soles (μ ≈ 2–4, verified sane in sim) | — |
| `walk.launch.py` / `stand.launch.py` | Same, minus gazebo include | Launch-tree surgery only |

## 6. Risks & open questions

1. **Mass is the whole design** (AT-11 finding §1.3): if the course requires the full
   1 kg payload AND a ≥ 8 kg platform, XM-class fails on continuous torque — escalate
   to the harmonic-drive upgrade path and re-run this document's §1.2 math with the
   real mass. Question §13 Q2 (mass target) gates this.
2. **Sim control tuning does not transfer numerically**: Dynamixel's internal position
   loop replaces the sim's effort PID; expect a new stiffness island hunt (the sim
   found the island is narrow). Budget 2–3 lab sessions for re-tuning; the sim's
   session-walking + settle_boost recipe is the starting point.
3. Budget question §13 Q4 unanswered: Tier A ≈ $5,600 all-in; Tier C ≈ $1,200 all-in
   (demonstrator). Decision needed before ordering.
4. Delivery lead times on XM540 (Robotis backorder history) — order first if chosen.

---

## 7. ★ SELECTED: Tier S — ₹15,000 student build (decision 2026-09-21)

User decision: **budget ₹15,000 INR (~$180), student project**. This rules out every
smart-serial-servo tier (one XM540 alone is ~₹42,000). Tier S is the closest-possible
build: a **desktop-scale (~1.6 kg) 10-DoF biped** that preserves the project's real
deliverable — the **control stack** (session-walking gait, settle_boost, IMU gates,
contact odometry, RViz) — while substituting the one thing ₹15k cannot buy: the
effort-feedback servo architecture.

### 7.1 Bill of materials (₹, verified street prices 2026-09)

| Item | Qty | Unit ₹ | Total ₹ | Source/notes |
|---|---|---|---|---|
| DS3218MG 20 kg·cm digital metal-gear servo | 10 | 1,100 | 11,000 | vendor spread ₹879 (REES52) – ₹1,999 (robu 270°); price at median |
| PCA9685 16-ch PWM servo driver | 1 | 369 | 369 | robu.in SKU 31364, verified |
| ESP32 DevKit WROOM-32 | 1 | 420 | 420 | robu/robogears, verified |
| MPU6050 IMU breakout | 1 | 150 | 150 | GY-521; BNO085 exceeds budget |
| FSR-402 force sensors (2/sole) | 4 | 30 | 120 | ADS1115 not needed — ESP32 ADC suffices for contact thresholding |
| LM2596 buck converter (6 V/5 A servo rail) | 1 | 150 | 150 | from 2S pack |
| 2S 18650 holder + 2× 2200 mAh cells + charger | 1 | 800 | 800 | bench PSU = salvaged laptop brick (₹0) |
| Frame: 5 mm ply / acrylic plates + M3 standoff kit | 1 | 400 | 400 | laser-cut or hand-cut |
| Wiring, headers, XT60, heat-shrink | — | — | 300 | |
| Contingency + shipping | — | — | 1,300 | spread + one spare servo |
| **Total** | | | **≈ ₹15,000** | |

### 7.2 Mass & torque budget (honest math)

- Robot ≤ **1.6 kg** total incl. 0.2 kg payload (pelvis 0.5, legs 2 × 0.45).
- Hip continuous moment ≈ 1.0 kg × 9.8 × 0.06 m ≈ **0.6 N·m**; with payload 0.3 N·m
  more → ~0.9 N·m vs DS3218 1.96 N·m stall. Duty ≈ 45 % of stall **in single support
  only** — acceptable *only* in the sim-proven **session-walking mode** (quasi-static
  10 s steps, settle between sessions). The AT-11 session result transfers as the
  duty-cycle prediction.
- Ankle/roll demands at this scale: ≤ 0.25 N·m — comfortable.

### 7.3 Architecture deltas (everything else in this repo unchanged)

| Sim component | Tier S counterpart | Change |
|---|---|---|
| gz_ros2_control GazeboSimSystem (position+effort) | **`pwm_servo_hw`** ros2_control plugin: POSITION command → PCA9685 µs over I²C; effort state = 0 (no torque feedback) | new `pwm_servo_hw.cpp` (this repo) |
| gz IMU sensor /imu | MPU6050 on ESP32 → serial 115200 → **`esp32_sensor_bridge`** republishing `sensor_msgs/Imu` | new `esp32_sensor_bridge.cpp` (this repo) |
| gz contact sensors → ros_gz Contacts | FSR ADC → ESP32 serial → bridge **synthesizes `ros_gz_interfaces/Contacts`** on /left_foot_contact, /right_foot_contact | gait_odometry + contact_markers **byte-identical** |
| JTC effort-mode per-joint PID | JTC **position mode** (servo's internal loop IS the plant); Kp gains become trajectory-feedforward only | controllers.yaml: use `p: 1.0` position gains |
| spawn hold_joints | PCA9685 sends the stand pose at boot | same behavior |
| URDF mass 8.4 kg | URDF rescaled ≤ 1.6 kg + `ros2_control_hw.xacro` | M13 work |
| E-stop (motor rail cut) | identical: toggle on 6 V servo rail, logic stays up | §12 preserved |

Known accepted losses (documented, not hidden): no effort feedback → the JTC's
tracking-error abort criterion degrades to position-error only; IMU 100 Hz → ~50 Hz
(MPU6050 default); servo sag ≈ the sim's 0.13 rad finding, re-tuned on hardware
(the stiffness-island hunt is expected to repeat — budget 2–3 lab sessions).
