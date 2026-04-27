#!/usr/bin/env bash
# run_yolo.sh - Launch the YOLO detector node inside the project container.
#
# Why this wrapper exists:
#   It bundles the env sourcing (ROS2 + workspace overlay), the sanity
#   checks (workspace built? PX4 / Gazebo running?), and a --model flag
#   for swapping weights without remembering the full --ros-args incantation.
#
#   torch / ultralytics / numpy<2 are installed system-wide by the project
#   Dockerfile, so we just call the system python3 — no venv anywhere.
#
# Usage:
#   run_yolo.sh                       # default model (yolov8n.pt)
#   run_yolo.sh --model yolov8s.pt    # override model weights
#   run_yolo.sh --help                # show this help

set -euo pipefail

# --- Path resolution (works regardless of cwd) ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="${REPO_ROOT}/ros2_ws"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

MODEL_PATH=""

usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Launch the yolo_detector ROS2 node. Run inside the project container,
where torch + ultralytics + numpy<2 are installed system-wide.

Options:
  --model PATH    YOLO weights file to load (default: yolov8n.pt).
                  Examples: yolov8n.pt, yolov8s.pt, yolov8m.pt
  -h, --help      Show this message and exit.

Prerequisites (the script will check these):
  1. PX4 SITL + Gazebo running with the x500_mono_cam airframe.
  2. ros2_ws built (install/setup.bash present).
  3. torch, ultralytics, cv_bridge importable from system python3 (the
     project Dockerfile installs them).

Example:
  $(basename "$0") --model yolov8s.pt
EOF
}

# --- Argument parsing ---
while [[ $# -gt 0 ]]; do
  case "$1" in
    --model)
      MODEL_PATH="${2:-}"
      if [[ -z "${MODEL_PATH}" ]]; then
        echo "ERROR: --model requires a path argument." >&2
        exit 2
      fi
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

fail() {
  echo "ERROR: $*" >&2
  exit 1
}

# --- Sanity checks ---

# 1. Workspace built?
if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  fail "ROS2 workspace not built (no ${WS_DIR}/install/setup.bash).
  Build it with:
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ${WS_DIR}
    colcon build"
fi

# 2. PX4 / Gazebo running? (best-effort — gz topic may not be in PATH)
if command -v gz >/dev/null 2>&1; then
  if ! gz topic -l 2>/dev/null | grep -q 'x500_mono_cam'; then
    echo "WARNING: x500_mono_cam not found in 'gz topic -l'." >&2
    echo "         Make sure PX4 SITL is running:" >&2
    echo "           cd /root/PX4-Autopilot && make px4_sitl gz_x500_mono_cam" >&2
    echo "         Continuing anyway in case gz CLI is shadowed..." >&2
  fi
else
  echo "NOTE: 'gz' not on PATH; skipping Gazebo liveness check." >&2
fi

# --- Set up environment and launch ---
# ROS setup scripts touch unset variables, so disable nounset across sourcing.
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${WS_DIR}/install/setup.bash"
set -u

# 3. Required Python modules importable? (cv_bridge needs the ROS env above.)
if ! python3 -c 'import torch, ultralytics, cv_bridge' >/dev/null 2>&1; then
  fail "torch / ultralytics / cv_bridge not importable from system python3.
  This script expects to run inside the project container built from
  docker/Dockerfile, which installs torch + ultralytics system-wide and
  exposes cv_bridge via /opt/ros/${ROS_DISTRO}/setup.bash."
fi

ENTRY_SCRIPT="${WS_DIR}/install/yolo_detector/lib/yolo_detector/detector_node"
if [[ ! -f "${ENTRY_SCRIPT}" ]]; then
  fail "Entry script ${ENTRY_SCRIPT} not found. Did 'colcon build' succeed for yolo_detector?"
fi

echo "Launching yolo_detector with $(command -v python3)"
ROS_ARGS=( --ros-args )
if [[ -n "${MODEL_PATH}" ]]; then
  echo "Model override: ${MODEL_PATH}"
  ROS_ARGS+=( -p "model_path:=${MODEL_PATH}" )
fi

# Invoke the entry script directly with python3 (matches the colcon-baked
# shebang; the explicit invocation just keeps the wrapper symmetric and
# makes it trivial to swap interpreters later if needed).
exec python3 "${ENTRY_SCRIPT}" "${ROS_ARGS[@]}"
