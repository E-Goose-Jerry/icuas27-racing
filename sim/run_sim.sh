#!/usr/bin/env bash
# Host-side driver for the simulation container.
#
#   ./sim/run_sim.sh build      build the image (slow: ROS + PX4 + LUS)
#   ./sim/run_sim.sh shell      interactive shell with ROS + LUS + racing layer
#   ./sim/run_sim.sh build-race colcon build the competition packages in-container
#   ./sim/run_sim.sh sim        start the full tmux session (Gazebo + PX4 + stack)
#   ./sim/run_sim.sh race       start only the race nodes (needs `sim` running)
#   ./sim/run_sim.sh dashboard  serve the ground station on :8080
#   ./sim/run_sim.sh test       run the unit tests inside the container
#   ./sim/run_sim.sh check      environment self-check (docker, GPU, X11)
#
# X11: Gazebo needs a display. The container joins the host X server through the
# Unix socket; no `xhost` is needed because the socket is mounted and the
# container runs as root with the same DISPLAY.
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ICUAS27_IMAGE:-icuas27-racing:local}"
CONTAINER_HOME="${ICUAS27_CONTAINER_HOME:-/opt/icuas27_racing}"
GPU_ARGS=()

log()  { printf '\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[!] %s\033[0m\n' "$*"; }
die()  { printf '\033[1;31m[x] %s\033[0m\n' "$*" >&2; exit 1; }

have() { command -v "$1" >/dev/null 2>&1; }

check() {
    have docker || die "docker is not installed"
    docker info >/dev/null 2>&1 || die "docker daemon not reachable (is your user in the docker group?)"

    if docker run --rm --gpus all ubuntu:22.04 true >/dev/null 2>&1; then
        log "GPU passthrough: available"
        GPU_ARGS=(--gpus all)
    else
        warn "GPU passthrough unavailable: Gazebo will render on the CPU (slow but usable)"
    fi

    if [ -n "${DISPLAY:-}" ] && [ -S /tmp/.X11-unix/X0 ]; then
        log "X11 display: ${DISPLAY}"
    else
        warn "no X11 socket detected: Gazebo will start headless and you will see nothing"
    fi

    if [ ! -f "${PROJECT_ROOT}/course/provisional_sample.json" ]; then
        warn "course files missing: run python3 tools/make_courses.py"
    fi
}

run_container() {
    local -a args=(
        --rm -it
        --net host
        --ipc host
        -e DISPLAY="${DISPLAY:-:0}"
        -e ICUAS27_UID="$(id -u)"
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
        -v "${PROJECT_ROOT}:${CONTAINER_HOME}:rw"
        -v "${PROJECT_ROOT}/../.cache/icuas27_bags:/tmp/icuas27_rosbags:rw"
        -w "${CONTAINER_HOME}"
    )
    docker run "${args[@]}" "${GPU_ARGS[@]}" "${IMAGE}" "$@"
}

build_image() {
    log "building ${IMAGE} (first build takes 40-60 min: ROS 2, PX4 SITL, acados, LUS)"
    docker build -t "${IMAGE}" "${PROJECT_ROOT}/sim/docker"
    log "image ready: ${IMAGE}"
}

build_race() {
    log "building the competition ROS packages inside the container"
    run_container bash -lc '
        set -e
        cd /opt/icuas27_racing/ros2_ws
        colcon build --symlink-install --packages-up-to icuas27_racing
        echo "built: $(ls install | tr "\n" " ")"
    '
}

start_session() {
    log "starting the simulation tmux session"
    run_container bash -lc '
        set -e
        cd /opt/icuas27_racing/sim/tmux
        ./start.sh
    '
}

start_race() {
    log "launching the race layer"
    run_container bash -lc '
        set -e
        source /opt/laser_uav_system_ws/install/setup.bash 2>/dev/null || true
        ros2 launch icuas27_racing race.launch.py \
            uav_name:="${UAV_NAME:-uav1}" \
            course_file:="${ICUAS27_COURSE:-/opt/icuas27_racing/course/provisional_sample.json}"
    '
}

case "${1:-check}" in
    check)       check ;;
    build)       check; build_image ;;
    shell)       run_container bash ;;
    build-race)  check; build_race ;;
    sim)         check; start_session ;;
    race)        check; start_race ;;
    dashboard)
        log "ground station: http://127.0.0.1:8080 (Ctrl-C to stop)"
        cd "${PROJECT_ROOT}" && exec python3 -m ground_station.server --host 0.0.0.0 --port 8080
        ;;
    test)
        log "unit tests: racing_core"
        ( cd "${PROJECT_ROOT}" && python3 -m unittest discover -s tests -t . )
        log "unit tests: ROS package"
        ( cd "${PROJECT_ROOT}/ros2_ws/src/icuas27_racing" && python3 -m pytest test -q )
        ;;
    *) die "unknown command '${1}'; try: check build shell build-race sim race dashboard test" ;;
esac
