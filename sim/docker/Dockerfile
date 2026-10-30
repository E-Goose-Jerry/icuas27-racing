# ICUAS 2027 racing simulation image.
#
# Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic + PX4 SITL, matching what the
# official LASER UAV System (LUS) requires. The host is Ubuntu 24.04 with ROS 2
# Jazzy, which LUS does not support, so the container is not a convenience: it
# is the only way to run the official stack unmodified.
#
# Build and run it through `sim/run_sim.sh`, not by hand.
#
# This image installs ROS 2, Gazebo and the build tooling itself, then hands the
# rest to the official `install.sh` in its CI branch (see bootstrap_lus.sh).
# The split is deliberate: the apt layer here is ours and cached, while the LUS
# build stays exactly the sequence the organisers test.
FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=humble \
    REAL_UAV=false \
    UAV_NAME=uav1 \
    UAV_TYPE=crazyflie \
    # install.sh selects its non-interactive CI branch on this variable
    GITHUB_ACTIONS=true \
    GITHUB_WORKSPACE=/opt \
    FCU=px4 \
    ROS_WS=/opt/laser_uav_system_ws \
    PYTHONPATH=/opt/icuas27_racing

SHELL ["/bin/bash", "-c"]

# -- base tooling ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl gnupg git git-lfs lsb-release sudo \
        build-essential cmake ninja-build pkg-config wget unzip xz-utils \
        python3 python3-pip python3-dev python3-venv python3-setuptools \
        tmux tmuxinator less vim jq toilet \
        libgl1 libglu1-mesa mesa-utils libgomp1 \
        libeigen3-dev libboost-all-dev libceres-dev \
        libunwind-dev libdw-dev libyaml-cpp-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    && rm -rf /var/lib/apt/lists/*

# -- ROS 2 Humble ------------------------------------------------------------
#
# Ubuntu 22.04 ships Gazebo Classic 11 as `gazebo` / `libgazebo-dev`. The
# versioned names (`gazebo11`, `libgazebo11-dev`) only exist in the OSRF apt
# repo and fail here with "Unable to locate package".
#
# PX4's SITL build needs jinja2/toml/yaml at configure time; the official
# install.sh does not install them, and the failure only shows up 20 minutes
# into the PX4 build -- hence they are here.
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-desktop ros-dev-tools \
        ros-humble-gazebo-ros-pkgs ros-humble-gazebo-msgs ros-humble-gazebo-plugins \
        ros-humble-cv-bridge ros-humble-image-transport ros-humble-eigen3-cmake-module \
        python3-colcon-common-extensions python3-colcon-clean \
        python3-opencv python3-numpy \
        gazebo libgazebo-dev \
        python3-jinja2 python3-toml python3-yaml python3-packaging \
    && rm -rf /var/lib/apt/lists/*

# empy must stay pinned: 4.x breaks the ROS 2 message generators
RUN python3 -m pip install --no-cache-dir --upgrade pip wheel \
    && python3 -m pip install --no-cache-dir \
        "gitman>=3,<4" "packaging==24.2" "empy==3.3.4" \
        setuptools kconfiglib jsonschema pyros pyros-genmsg future

# -- LASER UAV System: official installer, CI branch -------------------------
#
# Separate stage on purpose: `docker build --target base` gives an image with the
# whole (slow) apt layer, so the LUS build can be iterated in a container where
# the PX4 firmware and acados builds survive a failed attempt. Rebuilding this
# stage from scratch costs ~25 minutes; iterating in the container costs seconds.
FROM base AS lus

# NOTE: REAL_UAV deliberately keeps its plain `false` value here.
#
# Two consumers disagree about this variable, and both are load-bearing:
#   * install.sh compares against FALSE='"false"' in its CI branch, so it needs
#     the QUOTED form -- bootstrap_lus.sh exports that for the duration of the
#     install and nothing else.
#   * px4_api.launch.py builds a Python expression out of the environment
#     ('"' + REAL_UAV + '" == "false"'), so a quoted value is a syntax error and
#     the launch dies.
# The image therefore ships the plain value; only the build step overrides it.

COPY bootstrap_lus.sh /opt/bootstrap_lus.sh
RUN chmod +x /opt/bootstrap_lus.sh && /opt/bootstrap_lus.sh

# -- shell setup -------------------------------------------------------------
# install.sh runs `source ~/.bashrc` under `set -e`, so every line appended here
# must return 0. A bare `[ -f x ] && source x` returns 1 when the file is absent
# and would abort the installer.
RUN cat >> /root/.bashrc <<'EOF'

# ---- icuas27 racing container ----
source /opt/ros/humble/setup.bash
if [ -f /opt/laser_uav_system_ws/install/setup.bash ]; then
    source /opt/laser_uav_system_ws/install/setup.bash
fi
if [ -d /opt/laser_uav_system_ws/src/laser_uav_controllers/acados/lib ]; then
    export ACADOS_SOURCE_DIR=/opt/laser_uav_system_ws/src/laser_uav_controllers/acados
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ACADOS_SOURCE_DIR}/lib
fi
export PYTHONPATH=/opt/icuas27_racing:${PYTHONPATH}
export ICUAS27_PLATFORM=sim
export ICUAS27_PROJECT_ROOT=/opt/icuas27_racing
export ICUAS27_STATUS_FILE=/tmp/icuas27_race/status.json
EOF

# One environment file for every shell. `docker exec`, launch subprocesses and the
# entrypoint all need ROS + Gazebo paths; only the entrypoint used to get them,
# which made "works in entrypoint, empty in exec" the default failure mode.
COPY icuas27_env.sh /opt/icuas27_env.sh
ENV BASH_ENV=/opt/icuas27_env.sh

COPY entrypoint.sh /opt/entrypoint.sh
RUN chmod +x /opt/entrypoint.sh

WORKDIR /opt/icuas27_racing
ENTRYPOINT ["/opt/entrypoint.sh"]
CMD ["bash"]
