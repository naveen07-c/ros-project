#!/usr/bin/env bash
# Demo: biped climbs the stairs, walks 3 s onto the top platform, then halts
# at the end of a step (both feet planted) so it never walks off the far edge.
#
# Usage:
#   ./run.sh              # GUI demo
#   LOG=/tmp/demo.log ./run.sh
set -o pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DISPLAY="${DISPLAY:-:1}"
LOG="${LOG:-/tmp/biped_demo.log}"

source /opt/ros/jazzy/setup.bash
if [ ! -f "$DIR/install/setup.bash" ]; then
  echo "Building biped_gazebo_sim (first run)..."
  ( cd "$DIR" && colcon build --packages-select biped_gazebo_sim ) \
    || { echo "Build failed." >&2; exit 1; }
fi
source "$DIR/install/setup.bash"

# Remove a stale simulation (same set as the test harnesses).
ps -eo pid,args \
  | grep -iE "gz sim|gait_controller|simulation.launch|parameter_bridge|robot_state_pub|controller_manager" \
  | grep -v grep | awk '{print $1}' | xargs -r kill -9 2>/dev/null
sleep 3

echo "Starting biped stairs demo (log: $LOG)"
setsid ros2 launch biped_gazebo_sim simulation.launch.py \
  headless:=false walk:=true \
  world:="$DIR/ros2_gazebo_sim/worlds/stairs.sdf" \
  ankle_slope_sign:=-1.0 \
  edge_stop:=true edge_lookahead:=0.3 edge_drop:=0.15 \
  foot_lift:=0.04 kp_pitch:=0.0 kd_pitch:=0.0 \
  slope_speed_factor:=0.5 slope_speed_thresh:=0.03 slope_zmp_gain:=0.0 \
  speed:=0.1 step_time:=2.0 \
  "terrain_x:=0,1.5,1.5,1.7,1.7,1.9,1.9,2.1,2.1,2.3,2.3,8.0" \
  "terrain_z:=0,0,0.03,0.03,0.06,0.06,0.09,0.09,0.12,0.12,0.15,0.15" \
  walk_enable:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -TERM -"$LAUNCH_PID" 2>/dev/null
  wait "$LAUNCH_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

# Wait for the controller, then arm the demo stop: walk 3 s on the top
# platform and hold.  Retries until the node answers.
ok=0
for _ in $(seq 1 120); do
  if ros2 param set /gait_controller stop_after_climb 3.0 >/dev/null 2>&1; then
    ok=1
    break
  fi
  sleep 0.5
done
if [ "$ok" = 1 ]; then
  ros2 param set /gait_controller climb_z 0.14 >/dev/null 2>&1 || true
  echo "Demo stop armed: halts 3 s after reaching the top platform."
else
  echo "WARNING: could not reach /gait_controller; demo stop NOT armed." >&2
fi

echo "Watching for: CLIMBED / DEMO STOP in $LOG"
wait "$LAUNCH_PID"
