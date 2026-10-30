#!/usr/bin/env bash
# Headless smoke test: bring up the official stack without a Gazebo window.
#
# `gazebo.launch.py` starts gzserver and gzclient as two separate includes, so we
# can get a full simulation (physics, PX4 SITL, sensors, managers) with no window
# on the user's screen. Use this to validate the chain; use the tmux session when
# you actually want to watch the drone fly.
#
#   ./sim/smoke_test.sh            # full check, prints a verdict, tears down
#   ./sim/smoke_test.sh --keep     # leave everything running in container "smoke"
#   ./sim/smoke_test.sh --visual   # open the Gazebo window (needs an X display)
#
# Two things cost real debugging time here, so they are stated up front:
#   * `docker exec` does NOT run the image entrypoint, so every exec has to
#     source the workspaces itself (the ENV_SETUP preamble below).
#   * `pgrep -f gzserver` matches the shell running the check itself. Use
#     `pgrep -x` for exact process names.
set -uo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ICUAS27_IMAGE:-icuas27-racing:local}"
NAME="${ICUAS27_SMOKE_NAME:-smoke}"
KEEP=0
VISUAL=0
for arg in "$@"; do
    case "${arg}" in
        --keep)   KEEP=1 ;;
        --visual) VISUAL=1; KEEP=1 ;;
    esac
done

FAILS=0
log()  { printf '\033[1;34m==> %s\033[0m\n' "$*"; }
ok()   { printf '  \033[1;32m[ok]\033[0m   %s\n' "$*"; }
bad()  { printf '  \033[1;31m[fail]\033[0m %s\n' "$*"; FAILS=$((FAILS + 1)); }
warn() { printf '  \033[1;33m[warn]\033[0m %s\n' "$*"; }

# sourced by every exec; `set +u` because ROS setup scripts are not -u safe
ENV_SETUP='
set +u
source /opt/ros/humble/setup.bash
[ -f /opt/laser_uav_system_ws/install/setup.bash ] && source /opt/laser_uav_system_ws/install/setup.bash
[ -f /opt/icuas27_racing/ros2_ws/install/setup.bash ] && source /opt/icuas27_racing/ros2_ws/install/setup.bash
export PYTHONPATH=/opt/icuas27_racing:${PYTHONPATH:-}
export ICUAS27_STATUS_FILE=/tmp/icuas27_race/status.json
set -u
'

run() { docker exec "${NAME}" bash -c "${ENV_SETUP} $*"; }

docker rm -f "${NAME}" >/dev/null 2>&1 || true

if [ "${VISUAL}" -eq 1 ]; then log "starting the container (X11: ${DISPLAY})"; else log "starting the container (headless)"; fi
# root + HOME=/opt: LUS scripts expand `~/git/laser_uav_system`, and PX4 needs a
# passwd entry for the uid it runs as. See sim/run_sim.sh for the same note.
if [ "${VISUAL}" -eq 1 ]; then
    [ -n "${DISPLAY:-}" ] || { echo "no DISPLAY set; run from a graphical session" >&2; exit 1; }
    echo "  display: ${DISPLAY}"
    # X authorisation: mounting the socket is not enough, the container also
    # needs the cookie. Without it gzclient dies with
    # "Authorization required, but no authorization protocol specified".
    XAUTH_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
    local_auth=()
    if [ -f "${XAUTH_FILE}" ]; then
        local_auth=(-v "${XAUTH_FILE}:/root/.Xauthority:ro" -e XAUTHORITY=/root/.Xauthority)
    else
        echo "  [warn] no Xauthority file; gzclient may be refused by the X server" >&2
    fi
    docker run -d --name "${NAME}" --net host --ipc host \
        -e HOME=/opt -e DISPLAY="${DISPLAY}" \
        "${local_auth[@]}" \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "${PROJECT_ROOT}:/opt/icuas27_racing" \
        "${IMAGE}" sleep infinity >/dev/null
else
    docker run -d --name "${NAME}" --net host \
        -e HOME=/opt \
        -v "${PROJECT_ROOT}:/opt/icuas27_racing" \
        "${IMAGE}" sleep infinity >/dev/null
fi

cleanup() {
    if [ "${KEEP}" -eq 1 ]; then
        echo
        echo "stack left running in '${NAME}':"
        echo "  docker exec -it ${NAME} bash   # then source the two workspaces"
        echo "  docker rm -f ${NAME}           # stop and clean up"
    else
        docker rm -f "${NAME}" >/dev/null 2>&1 || true
    fi
}
trap cleanup EXIT

log "1. packages are visible"
run 'ros2 pkg prefix icuas27_racing >/dev/null 2>&1 && echo PKG_OK' | grep -q PKG_OK \
    && ok "icuas27_racing built" || bad "icuas27_racing not built (./sim/run_sim.sh build-race)"
run 'ros2 pkg prefix laser_uav_managers >/dev/null 2>&1 && echo LUS_OK' | grep -q LUS_OK \
    && ok "laser_uav_managers built" || bad "LUS not built"

if [ "${VISUAL}" -eq 1 ]; then
    log "2. gazebo with window (gzserver + gzclient, world crazyflie_map)"
    run 'nohup ros2 launch laser_uav_simulation gazebo.launch.py \
          world:=crazyflie_map real_time:=1.0 verbose:=false \
          > /tmp/gzserver.log 2>&1 &' >/dev/null 2>&1
else
    log "2. gzserver (physics only, no window)"
    run 'nohup ros2 launch gazebo_ros gzserver.launch.py \
          world:=/opt/git/laser_uav_system/ros_packages/laser_gazebo_resources/worlds/crazyflie_map.world \
          verbose:=false > /tmp/gzserver.log 2>&1 &' >/dev/null 2>&1
fi
sleep 10
run 'pgrep -x gzserver >/dev/null && echo GZ_RUNNING' | grep -q GZ_RUNNING \
    && ok "gzserver running" \
    || { bad "gzserver did not start"; run 'tail -20 /tmp/gzserver.log'; }

log "3. spawn the crazyflie (PX4 SITL + DDS bridge)"
run 'nohup ros2 launch laser_uav_simulation spawn_drones.launch.py \
      spawn_drones_file:=/opt/icuas27_racing/sim/configs/spawn.yaml > /tmp/spawn.log 2>&1 &' >/dev/null 2>&1
sleep 10
run 'pgrep -x px4 >/dev/null && echo PX4_RUNNING' | grep -q PX4_RUNNING \
    && ok "PX4 SITL process running" || { bad "PX4 SITL did not start"; run 'tail -25 /tmp/spawn.log'; }
run 'pgrep -x MicroXRCEAgent >/dev/null && echo AGENT_RUNNING' | grep -q AGENT_RUNNING \
    && ok "MicroXRCEAgent running (uXRCE-DDS bridge)" || warn "MicroXRCEAgent not seen"

log "4. flight stack"
for l in px4_api estimation_manager control_manager planning_manager; do
    pkg=laser_uav_managers
    [ "${l}" = "px4_api" ] && pkg=laser_uav_hw_api
    run "nohup ros2 launch ${pkg} ${l}.launch.py > /tmp/${l}.log 2>&1 &" >/dev/null 2>&1
done
sleep 15
run 'ros2 topic list' > /tmp/topics.txt 2>/dev/null
for t in /uav1/hw_api/odometry /uav1/estimation_manager/estimation \
         /uav1/control_manager/state /uav1/planning_manager/diagnostics; do
    grep -qx "${t}" /tmp/topics.txt && ok "topic ${t}" || bad "topic ${t} missing"
done

log "5. odometry is actually flowing"
if run 'timeout 10 ros2 topic hz /uav1/estimation_manager/estimation 2>/dev/null | grep -m1 "average rate"' > /tmp/hz.txt 2>&1; then
    ok "odometry: $(grep -m1 -o "average rate:.*" /tmp/hz.txt || echo "rate seen")"
else
    bad "no odometry rate (estimation_manager not publishing)"
fi

log "6. race node"
run 'nohup ros2 run icuas27_racing race_mission \
      --ros-args -p course_file:=/opt/icuas27_racing/course/arena_template.json \
      > /tmp/race.log 2>&1 &' >/dev/null 2>&1
sleep 8
run 'ros2 service list' 2>/dev/null | grep -q "/icuas27/race_control" \
    && ok "race_control service advertised" \
    || { bad "race node did not start"; run 'tail -25 /tmp/race.log'; }

STATUS_JSON=/tmp/icuas27_race/status.json
if run "cat ${STATUS_JSON}" > /tmp/status.txt 2>/dev/null && [ -s /tmp/status.txt ]; then
    ok "status document written ($(wc -c < /tmp/status.txt) bytes)"
else
    warn "no status document yet"
fi

echo
if [ "${FAILS}" -eq 0 ]; then
    printf '\033[1;32m  smoke test passed: the official stack runs headless and the race node is up.\033[0m\n'
else
    printf '\033[1;31m  %d check(s) failed.\033[0m\n' "${FAILS}"
fi
echo "logs inside the container: /tmp/gzserver.log /tmp/spawn.log /tmp/*.log /tmp/race.log"
echo "nothing here arms or takes off; a run starts with an explicit service call:"
echo "  ros2 service call /icuas27/race_control icuas27_interfaces/srv/RaceControl \"{command: start, reason: operator}\""

[ "${FAILS}" -eq 0 ]
