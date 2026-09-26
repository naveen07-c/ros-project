# Bipedal Payload-Carrying Robot — ROS 2 Jazzy + Gazebo Harmonic

Simulation and control stack for a **10-DoF bipedal robot** (2 legs × 5 DoF:
hip pitch/roll, knee, ankle pitch/roll) that carries a **1 kg payload** on a
platform above the pelvis. Built for ROS 2 **Jazzy** on Ubuntu 24.04 with
**Gazebo Harmonic**.

> Requirements, assumptions, and every acceptance-test verdict are tracked in
> [`PROJECT_REQUIREMENTS.md`](PROJECT_REQUIREMENTS.md) — the single source of
> truth. All invented dimensions/masses are explicitly tagged **ASSUMPTION**.

## Repository layout

```
PROJECT_REQUIREMENTS.md   requirements, ASSUMPTION tables, AT-01..AT-11 results log
HARDWARE_SELECTION.md     servo/actuator sizing from sim-measured torques; Tier S BOM
BRINGUP_PLAN.md           M13 physical bring-up plan (suspended-first, gated stages)
VALIDATION_PLAN.md        M14 hardware acceptance suite (1:1 mapping to sim ATs)
src/
├── biped_description/    URDF/Xacro model, RSP display launch, RViz config
├── biped_control/        ros2_control config, stand_hold, gait generator, odometry,
│                         sensor bridges, PWM-servo hardware plugin, ESP32 firmware
└── biped_bringup/        flat world SDF, full-stack Gazebo launch files
```

## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy (`/opt/ros/jazzy/setup.bash`)
- Gazebo Harmonic + `ros_gz` integration packages
- ros2_control stack:

```bash
sudo apt install ros-jazzy-gz-ros2-control \
     ros-jazzy-joint-trajectory-controller ros-jazzy-ros2controlcli
```

## Build

Packages live in this repo; the colcon workspace symlinks to it:

```bash
mkdir -p ~/RosProject_ws/src
ln -sfn /path/to/this/repo/src ~/RosProject_ws/src   # if not already linked
source /opt/ros/jazzy/setup.bash
cd ~/RosProject_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# M2 — model in RViz (URDF check, no physics)
ros2 launch biped_description display.launch.py

# M3/M4 — Gazebo only, headless spawn + standing free-fall settle
ros2 launch biped_bringup gazebo.launch.py          # use_gui:=true for GUI

# M5 — full standing stack: gazebo + controllers + stand_hold (15° crouch)
ros2 launch biped_control stand.launch.py           # with_payload:=true default

# M6+ — open-loop gait walking session
ros2 launch biped_control walk.launch.py            # see PROJECT_REQUIREMENTS §10.1
```

Quick health check once a stack is up:

```bash
ros2 control list_controllers      # joint_trajectory_controller should be active
ros2 topic hz /joint_states        # 500 Hz expected under the stand stack
```

## Verification status (2026-09-26 re-run)

| Acceptance test | Result |
|---|---|
| AT-01 requirements traceable | **PASS** |
| AT-02 URDF/Xacro valid (both payload variants, URDF→SDF friction merge) | **PASS** |
| AT-03 Gazebo spawn stands stable (z = 0.599999 m, 0 drift over 6 s) | **PASS** |
| AT-04 JTC + joint_states at 500 Hz | **PASS** |
| AT-05 crouch hold ≥ 30 s with/without 1 kg payload | **PASS** |
| AT-07 TF tree complete | **PASS** |
| AT-08 IMU + foot contacts | **PASS** |
| AT-09 RViz2 visualization | **PASS** |
| AT-10 gait stop halts safely | **PASS** |
| AT-06/AT-11 unattended 1 m walk | **Closed with characterized gap** — 3 clean steps/session reproducible; going further needs CoM/ZMP preview control (out of the open-loop scope chosen in §7) |

Full dated log with raw numbers: [`PROJECT_REQUIREMENTS.md` §10.1](PROJECT_REQUIREMENTS.md).

## Hardware phase (M12–M14)

`HARDWARE_SELECTION.md` derives servo torque requirements from the
sim-measured masses/torques and selects **Tier S — ₹15,000 student build**
(10 × DS3218 + ESP32 + PCA9685 + MPU6050 + FSRs). The hardware layer is
already implemented in `biped_control` (`pwm_servo_hw` ros2_control plugin,
`esp32_sensor_bridge`, ESP32 firmware, `ros2_control_hw.xacro`) so the sim
nodes run byte-identical on hardware. Execution of M13/M14 awaits the kit.

## Safety

No real motor commands during simulation development. Physical bring-up
(BRINGUP_PLAN.md) is bench-tested **suspended** until every gate passes; the
E-stop semantics (zero-trajectory + freeze) are identical in sim and hardware.
