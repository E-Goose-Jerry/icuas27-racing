#!/usr/bin/env bash
# Fetch and build the LASER UAV System (LUS) without any interactive prompts.
#
# Why not call install.sh directly from the Dockerfile? Because it has several
# `read -p` prompts (real uav? px4/ap? uav name? airframe?) and pipes are not a
# terminal, so it silently receives empty answers and then builds the wrong
# thing. But it ALSO ships a fully non-interactive branch for CI, selected by
# GITHUB_ACTIONS=true. That branch is the supported, tested path -- so this
# script drives it instead of reimplementing the 233 lines of setup.
#
# GITHUB_ACTIONS=true switches install.sh to:
#   BASE_DIR = $GITHUB_WORKSPACE
#   gitman install simulation $FCU --force      (positional groups, not --group)
#   skip every read -p
#   skip the real-UAV sensor SDK prompts
#
# What install.sh then does, and what a hand-rolled version would have missed:
#   * apt + pip dependencies (mavlink, pcl, ceres, empy==3.3.4, kconfiglib ...)
#   * gitman install of the px4 and simulation groups
#   * build_px4_firmware.sh  -> PX4 SITL binary (NOT a colcon package)
#   * Micro-XRCE-DDS-Agent   -> cmake build + install into /usr/local
#   * acados NMPC solver     -> needed by laser_uav_controllers
#   * GAZEBO_*/LD_LIBRARY_PATH for the px4 gazebo-classic plugins
#   * colcon build --symlink-install
#
# Anything that fails here is a real failure: install.sh runs under `set -e`,
# and so does this script.
set -euo pipefail

WORKSPACE="${GITHUB_WORKSPACE:-/opt}"
LUS_ROOT="${LUS_ROOT:-${WORKSPACE}/git/laser_uav_system}"
ROS_WS="${ROS_WS:-${WORKSPACE}/laser_uav_system_ws}"
LUS_REPO="${LUS_REPO:-https://github.com/LASER-Robotics/laser_uav_system.git}"
CF_REPO="${CF_REPO:-https://github.com/laser-robotics-research/laser_crazyflie.git}"
CF_DIR="${CF_DIR:-/opt/laser_crazyflie}"
BUILD_CORES="${BUILD_CORES:-$(($(nproc) / 2))}"
[[ ${BUILD_CORES} -lt 1 ]] && BUILD_CORES=1

export GITHUB_ACTIONS=true
export GITHUB_WORKSPACE="${WORKSPACE}"
# Quotes are part of the value on purpose: install.sh compares against
# FALSE="\"false\"" in its CI branch, and a plain `false` selects the
# real-hardware branch (no PX4 build, plus driver packages we do not need).
export REAL_UAV='"false"'
export FCU=px4
# Fail fast if the convention ever changes: install.sh's CI branch compares this
# against FALSE='"false"', and getting it wrong silently builds for real hardware.
if [ "${REAL_UAV}" != '"false"' ]; then
    echo "[x] REAL_UAV must be the quoted string \"false\", got [${REAL_UAV}]" >&2
    exit 1
fi
export MAKEFLAGS="-j${BUILD_CORES}"

log() { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }

# A stale workspace from a previous attempt would hide ABI/compile errors.
rm -rf "${ROS_WS}"

log "cloning LUS into ${LUS_ROOT}"
mkdir -p "$(dirname "${LUS_ROOT}")"
rm -rf "${LUS_ROOT}"
git clone --depth 1 -b stable "${LUS_REPO}" "${LUS_ROOT}"

# ROS must already be on the environment before install.sh runs. Its final step is
#     source ~/.bashrc && colcon build ...
# but Ubuntu's /root/.bashrc starts with `[ -z "$PS1" ] && return`, so in a
# non-interactive `bash -c` that source is a no-op and colcon dies with
# "ament_cmake not found" on the first message package. Sourcing it here makes
# install.sh's own build work.
log "sourcing ROS 2 Humble for install.sh"
# ROS setup scripts are not `set -u` safe (they reference $AMENT_TRACE_SETUP_FILES
# unbound), so -u has to come off for the duration of the source.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

log "running the official installer in its CI (non-interactive) branch"
cd "${LUS_ROOT}"
# install.sh runs under `set -e` and its last step is the colcon build. If that
# step fails we still want the artefacts check below to decide, and we rebuild
# ourselves at the end anyway -- so a non-zero exit here is not automatically
# fatal.
set +e
./install.sh
INSTALL_RC=$?
set -e
if [ "${INSTALL_RC}" -ne 0 ]; then
    echo "[warn] install.sh exited ${INSTALL_RC}; verifying the artefacts it should have produced"
fi

log "checking the pieces the racing layer depends on"
test -x "${LUS_ROOT}/ros_packages/px4_firmware/build/px4_sitl_default/bin/px4" \
    || { echo "[x] PX4 SITL binary missing: build_px4_firmware.sh did not run"; exit 1; }
test -d "${LUS_ROOT}/ros_packages/laser_uav_controllers/acados" \
    || { echo "[x] acados missing: NMPC control_manager cannot build"; exit 1; }
command -v MicroXRCEAgent >/dev/null \
    || { echo "[x] MicroXRCEAgent missing: the PX4 <-> ROS 2 bridge will not start"; exit 1; }
echo "  [ok] px4 sitl binary, MicroXRCEAgent and acados are in place"

log "overlaying the Crazyflie add-on"
rm -rf "${CF_DIR}"
git clone --depth 1 "${CF_REPO}" "${CF_DIR}"
LASER_UAV_SYSTEM="${LUS_ROOT}" bash "${CF_DIR}/install_crazyflie.sh"

log "rebuilding so the new crazyflie files are linked into the workspace"
cd "${ROS_WS}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
colcon build --symlink-install --parallel-workers "${BUILD_CORES}" \
    --cmake-args -DCMAKE_C_FLAGS="-Wno-error" -DCMAKE_CXX_FLAGS="-Wno-error" -Wno-dev

log "done: ${ROS_WS}/install/setup.bash"
ls -1 "${ROS_WS}/install" | head -30
