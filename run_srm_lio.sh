#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
config="srm"
bag=""
output=""
rviz="true"
rate="1.0"
offset="0.0"
rebuild=false

usage() {
  cat <<'USAGE'
Usage: bash run_srm_lio.sh [options]
  --config NAME|PATH    Sensor configuration (default: srm, Livox CustomMsg)
  --bag PATH            Replay a ROS 2 bag; omit for live sensor input
  --output PATH         New output directory (default: results/<timestamp>_<config>)
  --no-rviz             Run without RViz
  --rate NUMBER         Bag playback rate (default: 1.0)
  --start-offset SEC    Skip bag seconds before playback (default: 0.0)
  --build               Rebuild before running (first run builds automatically)
  -h, --help            Show this help

Source ROS 2 and any optional Livox driver overlay before running.
SRDA and IPMM always run together.
USAGE
}

while (($#)); do
  case "$1" in
    --config|--bag|--output|--rate|--start-offset)
      if (($# < 2)) || [[ -z "$2" || "$2" == --* ]]; then
        echo "Missing value for $1" >&2; exit 2
      fi
      case "$1" in
        --config) config="$2" ;;
        --bag) bag="$2" ;;
        --output) output="$2" ;;
        --rate) rate="$2" ;;
        --start-offset) offset="$2" ;;
      esac
      shift 2 ;;
    --no-rviz) rviz=false; shift ;;
    --build) rebuild=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! command -v ros2 >/dev/null 2>&1; then
  echo "Source your ROS 2 installation first (for example: source /opt/ros/humble/setup.bash)." >&2
  exit 3
fi
if [[ -f "$config" ]]; then config="$(realpath -- "$config")"; fi
if [[ -n "$bag" ]]; then
  [[ -e "$bag" ]] || { echo "Bag does not exist: $bag" >&2; exit 2; }
  bag="$(realpath -- "$bag")"
fi
if [[ -n "$output" ]]; then output="$(realpath -m -- "$output")"; fi
cd -- "$workspace"
if $rebuild || [[ ! -f install/srm_lio/share/srm_lio/package.xml ]]; then
  command -v colcon >/dev/null 2>&1 || { echo "Install colcon before building." >&2; exit 3; }
  colcon build --symlink-install --packages-select srm_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
source "$workspace/install/setup.bash"
actual_prefix="$(ros2 pkg prefix srm_lio)"
if [[ "$(realpath -- "$actual_prefix")" != "$(realpath -- "$workspace/install/srm_lio")" ]]; then
  echo "Another srm_lio installation is shadowing this workspace: $actual_prefix" >&2
  exit 3
fi
exec ros2 launch srm_lio lio_launch.py \
  "config:=$config" "bag:=$bag" "output_dir:=$output" \
  "rviz:=$rviz" "rate:=$rate" "start_offset:=$offset"
