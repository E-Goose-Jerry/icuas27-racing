"""Launch the race layer next to the official LASER UAV System stack.

This launch file starts the two competition nodes only. The flight stack
(Gazebo, PX4, hw_api, estimation/control/planning managers) is the official
one -- start it with the official tmux session, then bring this up::

    ros2 launch icuas27_racing race.launch.py course_file:=/abs/path/course.json

``racing_core`` must be importable. Either ``pip install -e .`` from the project
root once, or export ``PYTHONPATH=<project root>`` (what ``sim/run_race.sh``
does).
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_status_file() -> str:
    return os.environ.get("ICUAS27_STATUS_FILE", "/tmp/icuas27_race/status.json")


def generate_launch_description() -> LaunchDescription:
    uav_name = LaunchConfiguration("uav_name")
    course_file = LaunchConfiguration("course_file")
    platform = LaunchConfiguration("platform")
    auto_start = LaunchConfiguration("auto_start")
    image_topic = LaunchConfiguration("image_topic")
    status_file = LaunchConfiguration("status_file")
    publish_debug_image = LaunchConfiguration("publish_debug_image")

    race_params = {
        "uav_name": uav_name,
        "course_file": course_file,
        "platform": platform,
        "auto_start": auto_start,
        "status_file": status_file,
        "publish_rate": 10.0,
        "default_speed": 1.0,
        "lead": 1.5,
        "retry_limit": 2,
    }

    detector_params = {
        "uav_name": uav_name,
        "image_topic": image_topic,
        "publish_debug_image": publish_debug_image,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_name", default_value="uav1"),
            DeclareLaunchArgument(
                "course_file",
                default_value=os.environ.get("ICUAS27_COURSE", ""),
                description="absolute path to a course JSON; empty means auto-discover",
            ),
            DeclareLaunchArgument(
                "platform",
                default_value=os.environ.get("ICUAS27_PLATFORM", "sim"),
                description="sim or real: selects the stale-data budgets",
            ),
            DeclareLaunchArgument(
                "auto_start",
                default_value="false",
                description="begin the attempt automatically after a delay",
            ),
            DeclareLaunchArgument(
                "status_file", default_value=_default_status_file(),
                description="where the JSON contract is mirrored for the dashboard",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value=["/", uav_name, "/camera/ai_deck/image_raw"],
            ),
            DeclareLaunchArgument("publish_debug_image", default_value="false"),
            LogInfo(msg=["starting race layer; status file: ", status_file]),
            Node(
                package="icuas27_racing",
                executable="race_mission",
                name="race_mission",
                output="screen",
                parameters=[race_params],
            ),
            Node(
                package="icuas27_racing",
                executable="gate_detector",
                name="gate_detector",
                output="screen",
                parameters=[detector_params],
            ),
        ]
    )
