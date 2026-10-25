#!/usr/bin/env bash
# Run CAD walking/stairs by default; original physics modes remain available.
set -e
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-demo}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  stand|walk) package=biped_control ;;
  gazebo|demo) package=biped_bringup ;;
  *) echo "Usage: $0 [demo|stand|walk|gazebo] [ROS launch arguments...]" >&2; exit 2 ;;
esac
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-68}"
export GZ_PARTITION="${GZ_PARTITION:-ros_sim_cad}"
if [[ ! -f "$project_dir/install/setup.bash" ]]; then
  echo "Build first: cd \"$project_dir\" && colcon build --symlink-install" >&2
  exit 1
fi
source "$project_dir/install/setup.bash"
use_gui=false
if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then use_gui=true; fi
declare -a launch_args=()
for arg in "$@"; do
  if [[ "$arg" == use_gui:=* ]]; then use_gui="${arg#*=}"; else launch_args+=("$arg"); fi
done
exec ros2 launch "$package" "$mode.launch.py" "use_gui:=$use_gui" "${launch_args[@]}"
