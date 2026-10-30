# Runtime environment for the ICUAS 2027 simulation container.
#
# Sourced automatically by every non-interactive bash through BASH_ENV (set in
# the Dockerfile). This exists because `<cmd>` reached Docker in three different
# ways -- the entrypoint, `docker exec`, and launch subprocesses -- and only the
# first one inherited anything. Baking the environment here removes the whole
# class of "it works in the entrypoint but not in my exec" problems.
#
# `set +u` around the ROS setup scripts: they read variables that may be unset,
# and a `set -u` caller would abort on them.

set +u

# -- ROS 2 -------------------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# -- LASER UAV System --------------------------------------------------------
LUS_PREFIX=/opt/git/laser_uav_system/ros_packages
ROS_WS="${ROS_WS:-/opt/laser_uav_system_ws}"
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

# -- competition layer -------------------------------------------------------
if [ -f /opt/icuas27_racing/ros2_ws/install/setup.bash ]; then
    source /opt/icuas27_racing/ros2_ws/install/setup.bash
fi

# -- Gazebo Classic ----------------------------------------------------------
# /usr/share/gazebo/setup.bash sets GAZEBO_RESOURCE_PATH, which gzclient needs to
# find its shader library. Without it the GUI dies with
#   "Unable to find shader lib" -> "Failed to initialize scene" -> a Camera
#   assertion and a core dump, while gzserver is perfectly happy (the server does
#   not need shaders). The official installer only appends this to ~/.bashrc.
if [ -f /usr/share/gazebo/setup.bash ]; then
    source /usr/share/gazebo/setup.bash
fi
# The PX4 SITL plugins live outside the default search path. Without these two
# variables gzserver silently fails to load them, PX4 gets no sensor data, the
# EKF never initialises, and /uav1/hw_api/odometry simply never appears -- with
# no error anywhere. This is exactly how it presented the first time.
PX4_SITL_BUILD="${LUS_PREFIX}/px4_firmware/build/px4_sitl_default/build_gazebo-classic"
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}:${PX4_SITL_BUILD}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${PX4_SITL_BUILD}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}:\
${LUS_PREFIX}/px4_firmware/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:\
${LUS_PREFIX}/laser_uav_simulation/models:\
${LUS_PREFIX}/laser_uav_simulation/core/models"

# -- NMPC solver -------------------------------------------------------------
ACADOS_DIR="${ROS_WS}/src/laser_uav_controllers/acados"
if [ -d "${ACADOS_DIR}/lib" ]; then
    export ACADOS_SOURCE_DIR="${ACADOS_DIR}"
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ACADOS_DIR}/lib"
fi

# -- competition layer paths -------------------------------------------------
export PYTHONPATH="/opt/icuas27_racing:${PYTHONPATH:-}"
export ICUAS27_PROJECT_ROOT="${ICUAS27_PROJECT_ROOT:-/opt/icuas27_racing}"
export ICUAS27_PLATFORM="${ICUAS27_PLATFORM:-sim}"
export ICUAS27_STATUS_FILE="${ICUAS27_STATUS_FILE:-/tmp/icuas27_race/status.json}"
mkdir -p "$(dirname "${ICUAS27_STATUS_FILE}")" 2>/dev/null || true

# -- LUS conventions ---------------------------------------------------------
# HOME must be the directory that contains `git/laser_uav_system`, because the
# official spawn scripts expand `~/git/laser_uav_system/...` literally.
export HOME=/opt
export UAV_NAME="${UAV_NAME:-uav1}"
export UAV_TYPE="${UAV_TYPE:-crazyflie}"
