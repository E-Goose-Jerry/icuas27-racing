#!/usr/bin/env bash
# Bring the whole simulation up, in the order the official tmux session uses,
# with the two upstream gaps patched and each step verified before the next.
#
#   ./sim/bringup.sh            # start everything, print a verdict
#   ./sim/bringup.sh --visual   # same, but with the Gazebo window
#
# Why a script and not the tmux session: the ordering here is load-bearing.
#   * planning_manager must start AFTER control_manager, otherwise its lifecycle
#     activation silently fails and it rejects every request with
#     "fresh control state and odometry in the same frame are required"
#   * control_manager needs laser_uav_planners to exist as a package, otherwise
#     the launch dies before the node is even created
#
# Two upstream gaps are patched idempotently (both are one-liners, both are
# reported in docs/07): the missing package.xml of the planner parameter
# collection, and the missing control_state_out remap.
set -uo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ICUAS27_IMAGE:-icuas27-racing:local}"
NAME="${ICUAS27_SIM_NAME:-sim}"
VISUAL=0
[ "${1:-}" = "--visual" ] && VISUAL=1

log()  { printf '\033[1;34m==> %s\033[0m\n' "$*"; }
ok()   { printf '  \033[1;32m[ok]\033[0m   %s\n' "$*"; }
bad()  { printf '  \033[1;31m[fail]\033[0m %s\n' "$*"; }
warn() { printf '  \033[1;33m[warn]\033[0m %s\n' "$*"; }

docker rm -f "${NAME}" >/dev/null 2>&1 || true

log "starting container ${NAME}"
if [ "${VISUAL}" -eq 1 ]; then
    XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
    [ -f "${XAUTH_FILE}" ] || { bad "no Xauthority at ${XAUTH_FILE}"; exit 1; }
    # --device /dev/dri is not optional: without it gzclient falls back to
    # llvmpipe, eats ~9 cores and starves gzserver down to ~1 Hz.
    docker run -d --name "${NAME}" --net host --ipc host \
        --device /dev/dri \
        -e HOME=/opt -e DISPLAY="${DISPLAY:-:0}" -e XAUTHORITY=/root/.Xauthority \
        -v "${XAUTH_FILE}:/root/.Xauthority:ro" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "${PROJECT_ROOT}:/opt/icuas27_racing" \
        "${IMAGE}" sleep infinity >/dev/null
else
    docker run -d --name "${NAME}" --net host --ipc host \
        -e HOME=/opt \
        -v "${PROJECT_ROOT}:/opt/icuas27_racing" \
        "${IMAGE}" sleep infinity >/dev/null
fi

run() { docker exec -i "${NAME}" bash -s <<< "set +u; source /opt/ros/humble/setup.bash; source /opt/laser_uav_system_ws/install/setup.bash; source /opt/icuas27_racing/ros2_ws/install/setup.bash; set -u
$*"; }

log "1. patching the two upstream gaps (idempotent)"
run '
P=/opt/git/laser_uav_system/ros_packages/laser_uav_planners
if [ ! -f "$P/package.xml" ]; then
  printf "%s\n" "<?xml version=\"1.0\"?>" \
    "<package format=\"3\"><name>laser_uav_planners</name><version>0.0.0</version>" \
    "<description>Planner parameter collection.</description>" \
    "<maintainer email=\"noreply@example.com\">LUS</maintainer><license>BSD-3-Clause</license>" \
    "<buildtool_depend>ament_cmake</buildtool_depend>" \
    "<export><build_type>ament_cmake</build_type></export></package>" > "$P/package.xml"
  printf "%s\n" "cmake_minimum_required(VERSION 3.8)" "project(laser_uav_planners)" \
    "find_package(ament_cmake REQUIRED)" \
    "install(DIRECTORY params DESTINATION share/\${PROJECT_NAME})" "ament_package()" > "$P/CMakeLists.txt"
  echo "  added package.xml for laser_uav_planners"
fi
L=/opt/laser_uav_system_ws/install/laser_uav_managers/share/laser_uav_managers/launch/control_manager.launch.py
if ! grep -q "control_state_out" "$L"; then
  python3 - "$L" <<PYEOF
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
anchor = "(\x27/\x27 + uav_name + \x27/diagnostics_out\x27, \x27/\x27 + uav_name + \x27/control_manager/diagnostics\x27),"
assert anchor in s, "anchor not found"
s = s.replace(anchor, anchor + "\n            (\x27/\x27 + uav_name + \x27/control_state_out\x27, \x27/\x27 + uav_name + \x27/control_manager/state\x27),", 1)
p.write_text(s)
print("  added control_state_out remap")
PYEOF
  cd /opt/laser_uav_system_ws && colcon build --symlink-install --packages-select laser_uav_managers >/dev/null 2>&1
fi
cd /opt/laser_uav_system_ws && colcon build --symlink-install --packages-select laser_uav_planners >/dev/null 2>&1 || true
'

log "2. gazebo ($([ "${VISUAL}" -eq 1 ] && echo "with window" || echo "headless"))"
run 'nohup ros2 launch laser_uav_simulation gazebo.launch.py world:=crazyflie_map real_time:=1.0 verbose:=false > /tmp/gz.log 2>&1 &'
sleep 14
run 'pgrep -x gzserver >/dev/null && echo GZ_OK' | grep -q GZ_OK && ok "gzserver up" || bad "gzserver did not start"
[ "${VISUAL}" -eq 1 ] && { run 'pgrep -x gzclient >/dev/null && echo GC_OK' | grep -q GC_OK && ok "gzclient up" || warn "gzclient not running"; }

log "3. spawn the crazyflie"
run 'nohup ros2 launch laser_uav_simulation spawn_drones.launch.py spawn_drones_file:=/opt/icuas27_racing/sim/configs/spawn.yaml > /tmp/spawn.log 2>&1 &'
sleep 10
run 'pgrep -x px4 >/dev/null && echo PX4_OK' | grep -q PX4_OK && ok "PX4 SITL up" || bad "PX4 did not start"

log "4. flight stack (order matters)"
run 'nohup ros2 launch laser_uav_hw_api px4_api.launch.py > /tmp/px4_api.log 2>&1 &'
sleep 4
run 'nohup ros2 launch laser_uav_managers estimation_manager.launch.py > /tmp/estimation_manager.log 2>&1 &'
sleep 8
run 'nohup ros2 launch laser_uav_managers control_manager.launch.py > /tmp/control_manager.log 2>&1 &'
sleep 10
run 'nohup ros2 launch laser_uav_managers planning_manager.launch.py > /tmp/planning_manager.log 2>&1 &'
sleep 12

log "5. checks"
run 'ros2 topic hz /uav1/estimation_manager/estimation --window 20 2>/dev/null | grep -m1 "average rate"' \
  | grep -q "average rate" && ok "odometry flowing" || bad "no odometry"

for node in control_manager planning_manager; do
    state=$(run "ros2 lifecycle get /uav1/${node} 2>/dev/null")
    case "${state}" in
        *active*) ok "${node} ${state}" ;;
        *) bad "${node} state=${state:-unknown} (activation failed)" ;;
    esac
done

# the state topic the planner needs; if this is silent the planner rejects everything
if run 'ros2 topic hz /uav1/control_manager/state --window 20 2>/dev/null | grep -m1 "average rate"' | grep -q "average rate"; then
    ok "control state on /uav1/control_manager/state"
else
    bad "control state missing -- planner will reject every request"
fi

log "6. race node"
run 'nohup ros2 run icuas27_racing race_mission \
      --ros-args -p course_file:=/opt/icuas27_racing/course/arena_template.json > /tmp/race.log 2>&1 &'
sleep 8
run 'ros2 service list 2>/dev/null | grep -q icuas27/race_control && echo RC_OK' | grep -q RC_OK \
    && ok "race_control ready" || bad "race node did not start"

cat <<'TXT'

Now trigger the run when a human is present:

  docker exec -it sim bash -c '
    source /opt/ros/humble/setup.bash
    source /opt/icuas27_racing/ros2_ws/install/setup.bash
    ros2 service call /icuas27/race_control icuas27_interfaces/srv/RaceControl \
      "{command: start, reason: operator}"'

Watch it:

  docker exec -it sim bash -c 'cat /tmp/icuas27_race/status.json'
  # or the ground station: python3 -m ground_station.server

Stop everything:

  docker rm -f sim
TXT
