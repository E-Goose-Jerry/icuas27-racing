"""Race mission node: drives the LASER UAV System from the race state machine.

The node is deliberately thin. All decisions live in :mod:`racing_core`; this
file only

* reads the pose and the flight-controller state,
* feeds them to :class:`racing_core.mission_fsm.RaceMission`,
* turns the returned :class:`Action` objects into ``laser_msgs`` topics and
  ``std_srvs`` service calls,
* writes the versioned JSON contract to disk for the ground station.

Topic and service names follow the official LUS convention
(``/<uav_name>/planning_manager/trajectory_path`` and friends) so the stock
launch files need no remapping.
"""

from __future__ import annotations

import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:  # laser_msgs comes from the official LUS workspace
    from laser_msgs.msg import ApiPx4Diagnostics, PoseWithHeading, TrajectoryPath
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    raise SystemExit(
        "laser_msgs is not available. Source the LASER UAV System workspace first: "
        "`source ~/laser_uav_system_ws/install/setup.bash`"
    ) from exc

from icuas27_interfaces.msg import RaceStatus
from icuas27_interfaces.srv import RaceControl

from racing_core.geometry import Vec3
from racing_core.mission_fsm import Action, ActionKind, MissionError, RaceMission, State
from racing_core.protocol import build_document, write_document
from racing_core.safety import Level, SafetyLimits, SafetyMonitor
from racing_core.timing import RaceTimer

from .course_files import load_default_course, resolve_course


def _stamp_seconds(node: Node) -> float:
    """Simulation-aware clock as float seconds."""
    return node.get_clock().now().nanoseconds * 1e-9


class RaceNode(Node):
    """Mission driver for one vehicle."""

    def __init__(self) -> None:
        super().__init__("race_mission")

        # -- parameters ---------------------------------------------------
        self.uav_name = self.declare_parameter("uav_name", "uav1").value
        self.course_file = self.declare_parameter("course_file", "").value
        self.publish_rate = float(self.declare_parameter("publish_rate", 10.0).value)
        self.default_speed = float(self.declare_parameter("default_speed", 1.0).value)
        self.lead = float(self.declare_parameter("lead", 1.5).value)
        self.retry_limit = int(self.declare_parameter("retry_limit", 2).value)
        # Rulebook 3.2: a missed gate needs a legal retry, skipping does not
        # count as completion. Keep this false for anything that will be scored.
        self.allow_skip = bool(
            self.declare_parameter("allow_skip_with_penalty", False).value
        )
        self.auto_start = bool(self.declare_parameter("auto_start", False).value)
        self.auto_start_delay = float(self.declare_parameter("auto_start_delay", 5.0).value)
        self.status_file = str(
            self.declare_parameter("status_file", "/tmp/icuas27_race/status.json").value
        )
        platform = str(self.declare_parameter("platform", "sim").value)
        self.takeoff_height = float(self.declare_parameter("takeoff_height", 0.0).value)
        self.status_topic = str(
            self.declare_parameter("status_topic", "/icuas27/race_status_json").value
        )
        # planning_manager accepts a trajectory only when it has a fresh control
        # state AND fresh odometry in the same evaluation. A single publish from
        # an odometry callback therefore gets rejected with
        #   "fresh control state and odometry in the same frame are required"
        # so the current target is re-sent on a timer instead of once per event.
        self.republish_period = float(
            self.declare_parameter("republish_period", 1.0).value
        )
        # Grace window at start-up: "never received a heartbeat" is not the same
        # as "heartbeat is stale". Restarting this node while the vehicle is
        # already airborne used to abort instantly, because the first odometry
        # callback ran before any diagnostics had arrived.
        self.fcu_grace = float(self.declare_parameter("fcu_grace", 5.0).value)

        # -- course and mission -------------------------------------------
        try:
            self.course = load_default_course(self.course_file)
        except Exception as exc:  # noqa: BLE001 - reported through the node log
            self.get_logger().fatal(f"cannot load course: {exc}")
            raise
        self.get_logger().info(
            f"course '{self.course.name}': {self.course.gate_count} gates x "
            f"{self.course.laps} laps = {len(self.course.sequence())} passes "
            f"({resolve_course(self.course_file)})"
        )
        if self.takeoff_height <= 0.0:
            self.takeoff_height = float(self.course.takeoff_height)

        self.mission = RaceMission(
            course=self.course,
            timer=RaceTimer(),
            retry_limit=self.retry_limit,
            lead=self.lead,
            speed=self.default_speed,
            allow_skip_with_penalty=self.allow_skip,
        )
        self.safety = SafetyMonitor(
            limits=SafetyLimits(max_altitude=max(12.0, self.course.takeoff_height + 10.0)),
            platform=platform,
        )

        # -- ROS interfaces -----------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        base = f"/{self.uav_name}"
        self.odom_topic = f"{base}/estimation_manager/estimation"
        self.path_topic = f"{base}/planning_manager/trajectory_path"

        self.create_subscription(Odometry, self.odom_topic, self._on_odom, sensor_qos)
        # The safety monitor requires a fresh flight-controller heartbeat once
        # airborne. Without this subscription _last_fcu_at stayed None and the
        # first airborne tick aborted with "no flight-controller state received".
        self.create_subscription(
            ApiPx4Diagnostics,
            f"{base}/hw_api/diagnostics",
            self._on_fcu_diagnostics,
            10,
        )

        self.path_pub = self.create_publisher(TrajectoryPath, self.path_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.race_status_pub = self.create_publisher(RaceStatus, "/icuas27/race_status", 10)

        self.takeoff_client = self.create_client(Trigger, f"{base}/control_manager/takeoff")
        self.land_client = self.create_client(Trigger, f"{base}/control_manager/land")
        self.cancel_client = self.create_client(Trigger, f"{base}/planning_manager/cancel")
        self.arm_client = self.create_client(Trigger, f"{base}/hw_api/arm")

        self.create_service(RaceControl, "/icuas27/race_control", self._on_control)

        # -- state ---------------------------------------------------------
        self._last_pose: Vec3 | None = None
        self._last_pose_at: float | None = None
        self._last_fcu_at: float | None = None
        self._fcu_armed = False
        self._airborne = False
        self._seq = 0
        self._started_at: float | None = None
        self._last_status_write = 0.0
        self._abort_latched = False
        self._stuck_logged = False
        self._last_path_publish = 0.0
        self._started_at = _stamp_seconds(self)

        period = 1.0 / max(1.0, self.publish_rate)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"race_mission ready: pose={self.odom_topic} path={self.path_topic} "
            f"platform={self.safety.platform} auto_start={self.auto_start}"
        )

    # -- callbacks --------------------------------------------------------
    def _on_fcu_diagnostics(self, message: ApiPx4Diagnostics) -> None:
        """Timestamp the flight-controller heartbeat for the safety monitor."""
        self._last_fcu_at = _stamp_seconds(self)
        self._fcu_armed = bool(message.armed)

    def _on_odom(self, message: Odometry) -> None:
        now = _stamp_seconds(self)
        position = message.pose.pose.position
        pose = Vec3(float(position.x), float(position.y), float(position.z))
        self._last_pose = pose
        self._last_pose_at = now
        self._airborne = pose.z > max(0.2, 0.8 * self.takeoff_height)

        verdict = self.safety.check(
            now,
            pose_at=self._last_pose_at,
            pose=pose,
            fcu_at=self._last_fcu_at,
            require_fcu=self._airborne
            and (
                self._last_fcu_at is not None
                or now - self._started_at > self.fcu_grace
            ),
            airborne=self._airborne,
        )
        if verdict.level is Level.ABORT and not self._abort_latched:
            self._abort_latched = True
            self.get_logger().error(f"safety abort: {verdict.detail}")
            self._dispatch(self.mission.safe_stop(verdict.detail, now), now)
            return

        try:
            actions = self.mission.on_pose(now, pose)
        except MissionError as exc:
            self.get_logger().error(f"mission rejected pose: {exc}")
            return
        if self.mission.stuck and not self._stuck_logged:
            gate = self.mission.target_gate()
            self.get_logger().warning(
                f"stuck on gate {getattr(gate, 'id', '?')} after "
                f"{self.mission.attempts} attempts: retrying until it is passed "
                f"(rulebook 3.2; skipping would void the run)"
            )
            self._stuck_logged = True
        elif not self.mission.stuck:
            self._stuck_logged = False
        self._last_path_publish = 0.0
        self._started_at = _stamp_seconds(self)
        self._dispatch(actions, now)

    def _tick(self) -> None:
        now = _stamp_seconds(self)

        # Takeoff completes once the vehicle has actually climbed.
        if self.mission.state is State.TAKEOFF and self._airborne:
            self.get_logger().info(f"airborne at {self._last_pose}, starting the clock")
            self._dispatch(self.mission.on_takeoff_complete(now), now)

        # Landing completes when the vehicle is back on the ground.
        if self.mission.state is State.LANDING and self._last_pose is not None:
            if self._last_pose.z < 0.15:
                self._dispatch(self.mission.on_landed(now), now)

        if self.auto_start and self.mission.state is State.IDLE:
            if self._started_at is None:
                self._started_at = now
            elif now - self._started_at >= self.auto_start_delay:
                self.get_logger().info("auto_start: beginning the attempt")
                self._handle_start(now)

        self._republish_target(now)
        self._publish_status(now)

    def _republish_target(self, now: float) -> None:
        """Keep the planner fed with the current gate target.

        The planner rejects requests it cannot evaluate synchronously, and the
        controller needs a reference that never expires (0.5 s timeout), so the
        target is refreshed on a timer rather than only when the state machine
        changes. Gated on ``republish_period`` to stay cheap.
        """
        gate = self.mission.target_gate()
        if gate is None:
            return
        if now - self._last_path_publish < self.republish_period:
            return
        action = self.mission.target_action()
        if action is not None:
            self._publish_path(action)

    def _on_control(self, request: RaceControl.Request, response: RaceControl.Response):
        command = (request.command or "").strip().lower()
        now = _stamp_seconds(self)
        try:
            if command == "start":
                if self.mission.state is not State.IDLE:
                    response.accepted = False
                    response.message = f"cannot start from {self.mission.state.value}"
                else:
                    self._handle_start(now)
                    response.accepted = True
                    response.message = "takeoff commanded"
            elif command == "abort":
                self._dispatch(self.mission.abort(request.reason or "operator", now), now)
                response.accepted = True
                response.message = "aborting: cancel + land"
            elif command == "safe_stop":
                self._dispatch(
                    self.mission.safe_stop(request.reason or "operator", now), now
                )
                self._abort_latched = True
                response.accepted = True
                response.message = "safety stop: cancel + hold"
            elif command == "reset":
                if self.mission.timer.running:
                    self.mission.timer.stop(now)
                self.mission = RaceMission(
                    course=self.course,
                    timer=RaceTimer(),
                    retry_limit=self.retry_limit,
                    lead=self.lead,
                    speed=self.default_speed,
                    allow_skip_with_penalty=self.allow_skip,
                )
                self._abort_latched = False
                self._started_at = None
                response.accepted = True
                response.message = "mission reset to IDLE"
            else:
                response.accepted = False
                response.message = f"unknown command '{command}'"
        except MissionError as exc:
            response.accepted = False
            response.message = str(exc)
        response.state = self.mission.state.value
        self._publish_status(now)
        return response

    # -- actions ----------------------------------------------------------
    def _handle_start(self, now: float) -> None:
        actions = self.mission.start(now)
        self._dispatch(actions, now)

    def _dispatch(self, actions: tuple[Action, ...], now: float) -> None:
        for action in actions:
            self.get_logger().info(f"action: {action.kind.value} {action.detail}")
            if action.kind in (ActionKind.FLY_GATE, ActionKind.FLY_LANDING):
                self._publish_path(action)
            elif action.kind is ActionKind.START_TAKEOFF:
                self._call_trigger(self.arm_client, "arm")
                self._call_trigger(self.takeoff_client, "takeoff")
            elif action.kind is ActionKind.LAND:
                self._call_trigger(self.land_client, "land")
            elif action.kind is ActionKind.CANCEL_PLANNING:
                self._call_trigger(self.cancel_client, "cancel")
            elif action.kind is ActionKind.HOLD:
                self._call_trigger(self.cancel_client, "hold")

    def _publish_path(self, action: Action) -> None:
        message = TrajectoryPath()
        message.stop_on_waypoints = False
        message.speed = float(action.speed or self.default_speed)
        for point in action.waypoints:
            waypoint = PoseWithHeading()
            waypoint.position.x = float(point.x)
            waypoint.position.y = float(point.y)
            waypoint.position.z = float(point.z)
            waypoint.heading = float(action.heading if action.heading is not None else 0.0)
            message.waypoints.append(waypoint)
        self.path_pub.publish(message)
        self._last_path_publish = _stamp_seconds(self)
        self.get_logger().debug(
            f"published {len(message.waypoints)} waypoints at {message.speed:.2f} m/s"
        )

    def _call_trigger(self, client, label: str) -> None:
        if not client.service_is_ready():
            self.get_logger().warning(f"{label}: service not ready, request dropped")
            return

        def _done(future) -> None:
            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001 - surfaced in the log
                self.get_logger().error(f"{label} failed: {exc}")
                return
            level = self.get_logger().info if result.success else self.get_logger().warning
            level(f"{label}: success={result.success} message={result.message!r}")

        client.call_async(Trigger.Request()).add_done_callback(_done)

    # -- reporting --------------------------------------------------------
    def _publish_status(self, now: float) -> None:
        self._seq += 1
        passed = self.mission.index
        target = self.mission.target_gate()
        gates = []
        for gate in self.course.gates:
            gates.append(
                {
                    "id": gate.id,
                    "target": target is not None and gate.id == target.id,
                    "passed": self._gate_passes(gate.id) > 0,
                }
            )
        document = build_document(
            seq=self._seq,
            stamp=now,
            race=self.mission.status(now),
            safety=self.safety.to_dict(),
            course=self.course,
            gates=gates,
            detection={"source": "gate_detector", "visible": None},
        )
        payload = json.dumps(document, ensure_ascii=False)
        self.status_pub.publish(String(data=payload))

        if self.status_file and now - self._last_status_write >= 0.2:
            self._last_status_write = now
            try:
                write_document(self.status_file, document)
            except OSError as exc:
                self.get_logger().warning(f"cannot write status file: {exc}")

        status = RaceStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = document["race"]["state"]
        status.gate_index = passed
        status.total_gates = self.mission.total_gates
        status.target_gate_id = 0 if target is None else target.id
        timer = document["race"]["timer"]
        status.elapsed = -1.0 if timer["elapsed"] is None else float(timer["elapsed"])
        status.adjusted = -1.0 if timer["adjusted"] is None else float(timer["adjusted"])
        status.penalty_count = len(timer["penalties"])
        status.incomplete = bool(self.mission.incomplete)
        self.race_status_pub.publish(status)

    def _gate_passes(self, gate_id: int) -> int:
        return sum(1 for passed_id, _ in self.mission.timer.gate_times if passed_id == gate_id)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node: RaceNode | None = None
    try:
        node = RaceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001 - last line of defence in flight
        if node is not None:
            node.get_logger().fatal(f"race_mission crashed: {exc}")
        else:
            print(f"race_mission failed to start: {exc}", flush=True)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
