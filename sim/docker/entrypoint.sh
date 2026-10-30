#!/usr/bin/env bash
# Container entrypoint: set up ROS and the competition layer, then hand over.
set -e

source /opt/ros/humble/setup.bash
if [ -f "${ROS_WS:-/opt/laser_uav_system_ws}/install/setup.bash" ]; then
    # LUS prints harmless warnings when a package was skipped; keep them quiet
    # so they do not look like the reason something failed.
    source "${ROS_WS:-/opt/laser_uav_system_ws}/install/setup.bash" 2>/dev/null || true
fi

# The competition layer lives at /opt/icuas27_racing (bind-mounted by run_sim.sh).
export PYTHONPATH="/opt/icuas27_racing:${PYTHONPATH:-}"
export ICUAS27_PROJECT_ROOT="${ICUAS27_PROJECT_ROOT:-/opt/icuas27_racing}"
export ICUAS27_PLATFORM="${ICUAS27_PLATFORM:-sim}"
export ICUAS27_STATUS_FILE="${ICUAS27_STATUS_FILE:-/tmp/icuas27_race/status.json}"
mkdir -p "$(dirname "${ICUAS27_STATUS_FILE}")"

if [ -f "${ICUAS27_PROJECT_ROOT}/ros2_ws/install/setup.bash" ]; then
    source "${ICUAS27_PROJECT_ROOT}/ros2_ws/install/setup.bash"
fi

exec "$@"
