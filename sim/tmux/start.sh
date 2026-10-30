#!/usr/bin/env bash
# Start the competition tmux session (mirrors laser_uav_system/tmux/*/start.sh).
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

if ! command -v tmuxinator >/dev/null 2>&1; then
    echo "[x] tmuxinator is not installed (apt install tmuxinator)" >&2
    exit 1
fi
if ! command -v ros2 >/dev/null 2>&1; then
    echo "[x] ROS 2 is not on PATH; source /opt/ros/humble/setup.bash first" >&2
    exit 1
fi

# tmuxinator insists on .tmuxinator.yml
rm -f .tmuxinator.yml
ln -s race_session.yml .tmuxinator.yml
exec tmuxinator start -p race_session.yml
