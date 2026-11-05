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
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:  # laser_msgs comes from the official LUS workspace
    from laser_msgs.msg import ApiPx4Diagnostics, PoseWithHeading, TrajectoryPath, UavControlDiagnostics
except ImportError as exc:  # pragma: no cover - environment problem, not logic
    raise SystemExit(
        "laser_msgs is not available. Source the LASER UAV System workspace first: "
        "`source ~/laser_uav_system_ws/install/setup.bash`"
    ) from exc

from icuas27_interfaces.msg import GateObservation, RaceStatus
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
        self.lus_backend = str(self.declare_parameter("lus_backend", "planning_manager").value)
        if self.lus_backend not in ("planning_manager", "control_manager"):
            raise ValueError("lus_backend must be planning_manager or control_manager")
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
        self.path_topic = f"{base}/{self.lus_backend}/trajectory_path"

        self.create_subscription(Odometry, self.odom_topic, self._on_odom, sensor_qos)
        self.create_subscription(ApiPx4Diagnostics, f"{base}/hw_api/diagnostics", self._on_fcu, sensor_qos)
        self.create_subscription(UavControlDiagnostics, f"{base}/control_manager/diagnostics", self._on_flight, sensor_qos)
        self.create_subscription(GateObservation, "/icuas27/gate_observation", self._on_detection, sensor_qos)

        self.path_pub = self.create_publisher(TrajectoryPath, self.path_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.race_status_pub = self.create_publisher(RaceStatus, "/icuas27/race_status", 10)

        self.takeoff_client = self.create_client(Trigger, f"{base}/control_manager/takeoff")
        self.land_client = self.create_client(Trigger, f"{base}/control_manager/land")
        self.cancel_client = self.create_client(Trigger, f"{base}/planning_manager/cancel")
        self.arm_client = self.create_client(Trigger, f"{base}/hw_api/arm")
        self.disarm_client = self.create_client(Trigger, f"{base}/hw_api/disarm")

        self.create_service(RaceControl, "/icuas27/race_control", self._on_control)

        # -- state ---------------------------------------------------------
        self._last_pose: Vec3 | None = None
        self._last_pose_at: float | None = None
        self._last_fcu_at: float | None = None
        self._airborne = False
        self._seq = 0
        self._started_at: float | None = None
        self._last_status_write = 0.0
        self._abort_latched = False
        self._stuck_logged = False
        self._armed = False
        self._offboard = False
        self._preflight = False
        self._control_flying = False
        self._last_control_at: float | None = None
        self._vertical_speed = 0.0
        self._speed = math.inf
        self._last_heading = 0.0
        self._arm_started_at: float | None = None
        self._arm_ack = False
        self._takeoff_requested = False
        self._landing_target: Vec3 | None = None
        self._land_requested = False
        self._disarm_requested = False
        self._last_detection = {"source": "gate_detector", "visible": None}
        self._last_path_publish = 0.0

        period = 1.0 / max(1.0, self.publish_rate)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"race_mission ready: pose={self.odom_topic} path={self.path_topic} "
            f"platform={self.safety.platform} auto_start={self.auto_start}"
        )

    # -- callbacks --------------------------------------------------------
    def _on_odom(self, message: Odometry) -> None:
        now = _stamp_seconds(self)
        position = message.pose.pose.position
        pose = Vec3(float(position.x), float(position.y), float(position.z))
        self._last_pose = pose
        self._last_pose_at = now
        self._airborne = pose.z > max(0.2, 0.8 * self.takeoff_height)
        self._vertical_speed = float(message.twist.twist.linear.z)
        velocity = message.twist.twist.linear
        self._speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        q = message.pose.pose.orientation
        self._last_heading = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

        if not self._check_safety(now):
            return

        try:
            actions = self.mission.on_pose(now, pose)
        except MissionError as exc:
            self.get_logger().error(f"mission rejected pose: {exc}")
            return
        if self.mission.stuck and not self._stuck_logged:
            gate = self.mission.target_gate()
            self.get_logger().warning(f"stuck on gate {getattr(gate, 'id', '?')}: retrying")
            self._stuck_logged = True
        elif not self.mission.stuck:
            self._stuck_logged = False
        self._dispatch(actions, now)

    def _on_fcu(self, message: ApiPx4Diagnostics) -> None:
        self._last_fcu_at = _stamp_seconds(self)
        self._armed = bool(message.armed)
        self._offboard = bool(message.offboard_mode)
        self._preflight = bool(message.preflight_checks_passed)

    def _on_flight(self, message: UavControlDiagnostics) -> None:
        self._last_control_at = _stamp_seconds(self)
        self._control_flying = bool(message.is_fly)

    def _on_detection(self, message: GateObservation) -> None:
        # Report vision independently; navigation still uses course geometry.
        self._last_detection = dict(source=message.source, visible=bool(message.visible),
                                    confidence=float(message.confidence),
                                    stamp=_stamp_seconds(self))

    def _check_safety(self, now: float) -> bool:
        verdict = self.safety.check(
            now,
            pose_at=self._last_pose_at,
            pose=self._last_pose,
            fcu_at=self._last_fcu_at,
            require_fcu=self._armed or self._airborne,
            airborne=self._airborne or self._control_flying,
        )
        if verdict.level is Level.ABORT and not self._abort_latched:
            self._abort_latched = True
            self.get_logger().error(f"safety abort: {verdict.detail}")
            self._dispatch(self.mission.safe_stop(verdict.detail, now), now)
        return verdict.level is not Level.ABORT

    def _tick(self) -> None:
        now = _stamp_seconds(self)
        safe = self._check_safety(now)

        if self.mission.state is State.TAKEOFF and safe:
            if self._arm_ack and self._armed and self._offboard and not self._takeoff_requested:
                self._takeoff_requested = True
                self._call_trigger(self.takeoff_client, "takeoff", required=True)
            elif self._arm_started_at is not None and not self._takeoff_requested and now-self._arm_started_at > 10:
                self._fail_start("arming/offboard confirmation timed out")

        # Takeoff completes once the vehicle has actually climbed.
        if (self.mission.state is State.TAKEOFF and safe and self._airborne
                and self._takeoff_requested and self._control_flying
                and self._last_pose is not None
                and abs(self._last_pose.z-self.takeoff_height) < .15
                and abs(self._vertical_speed) < .1
                and self._last_control_at is not None and now-self._last_control_at < 1):
            self.get_logger().info(f"airborne at {self._last_pose}, starting the clock")
            self._dispatch(self.mission.on_takeoff_complete(now), now)

        if self.mission.state is State.LANDING and self._last_pose is not None:
            if self._landing_target is not None and not self._land_requested:
                # The LUS land service captures the current XY, not the requested
                # landing target. Wait for the stopping path to settle before descent.
                if (safe and self._last_pose_at is not None and now-self._last_pose_at < .5
                        and (self._last_pose-self._landing_target).norm() < .1
                        and self._speed < .1):
                    self._request_land()
            if (self._land_requested and not self._control_flying
                    and self._last_pose.z < .3 and abs(self._vertical_speed) < .1):
                if self._armed and not self._disarm_requested:
                    self._disarm_requested = True
                    self._call_trigger(self.disarm_client, "disarm", required=True)
                elif not self._armed:
                    self._dispatch(self.mission.on_landed(now), now)

        if self.auto_start and self.mission.state is State.IDLE:
            if self._started_at is None:
                self._started_at = now
            elif now - self._started_at >= self.auto_start_delay and not self._start_problem(now):
                self.get_logger().info("auto_start: beginning the attempt")
                self._handle_start(now)

        if self.lus_backend == "planning_manager":
            self._republish_target(now)
        self._publish_status(now)

    def _republish_target(self, now: float) -> None:
        # Retain upstream freshness retry only for the newer planner. Re-sending
        # a legacy control-manager path would restart its trajectory.
        if self.mission.target_gate() is None or now-self._last_path_publish < self.republish_period:
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
                elif self._start_problem(now):
                    response.accepted = False
                    response.message = self._start_problem(now)
                else:
                    self._handle_start(now)
                    response.accepted = True
                    response.message = "arming requested; waiting for Armed/Offboard before takeoff"
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
                if self._armed or self._control_flying or self.mission.state is State.TAKEOFF:
                    raise MissionError("cannot reset while armed or flying")
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
                self._arm_started_at = None
                self._arm_ack = False
                self._takeoff_requested = False
                self._landing_target = None
                self._land_requested = False
                self._disarm_requested = False
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
        problem = self._start_problem(now)
        if problem:
            raise MissionError(problem)
        actions = self.mission.start(now)
        self._dispatch(actions, now)

    def _start_problem(self, now: float) -> str:
        if self._last_pose is None or not self._last_pose.is_finite() or self._last_pose_at is None or now-self._last_pose_at > self.safety.pose_timeout:
            return "no fresh finite pose; wait for estimation"
        if not self.safety.limits.inside_bounds(self._last_pose) or self._last_pose.z > self.safety.limits.max_altitude:
            return "pose outside configured flight limits"
        if self._last_fcu_at is None or now-self._last_fcu_at > self.safety.fcu_timeout or not self._preflight:
            return "flight-controller preflight/heartbeat not ready"
        if self._armed or self._control_flying or self._airborne:
            return "vehicle must be landed and disarmed before start"
        if self._last_control_at is None or now-self._last_control_at > 1:
            return "control-manager diagnostics not ready"
        if not all(c.service_is_ready() for c in (self.arm_client, self.takeoff_client, self.land_client, self.disarm_client)):
            return "flight services not ready"
        if self.path_pub.get_subscription_count() == 0:
            return f"no subscriber for {self.path_topic}"
        if self.lus_backend == "planning_manager" and not self.cancel_client.service_is_ready():
            return "planning-manager cancel service not ready"
        return ""

    def _fail_start(self, detail: str) -> None:
        self.get_logger().error(detail)
        self._abort_latched = True
        self._dispatch(self.mission.abort(detail, _stamp_seconds(self)), _stamp_seconds(self))

    def _request_land(self) -> None:
        if not self._land_requested:
            self._land_requested = True
            self._call_trigger(self.land_client, "land", required=True)

    def _hold(self) -> None:
        if self.lus_backend == "planning_manager":
            self._call_trigger(self.cancel_client, "cancel", required=True)
        elif self._control_flying and self._last_pose is not None:
            # Legacy LUS has no cancel service. Replacing the trajectory with
            # a one-point stopping path also works at the current position;
            # its goto callback rejects a target closer than 0.1 m.
            msg = TrajectoryPath(stop_on_waypoints=True, speed=.3)
            point = PoseWithHeading()
            point.position.x = float(self._last_pose.x)
            point.position.y = float(self._last_pose.y)
            point.position.z = float(self._last_pose.z)
            point.heading = self._last_heading
            msg.waypoints.append(point)
            self.path_pub.publish(msg)
        elif self._armed and self._takeoff_requested:
            self._request_land()

    def _dispatch(self, actions: tuple[Action, ...], now: float) -> None:
        for action in actions:
            self.get_logger().info(f"action: {action.kind.value} {action.detail}")
            if action.kind in (ActionKind.FLY_GATE, ActionKind.FLY_LANDING):
                self._publish_path(action)
                if action.kind is ActionKind.FLY_LANDING:
                    self._landing_target = action.waypoints[-1]
            elif action.kind is ActionKind.START_TAKEOFF:
                self._arm_started_at = now
                self._arm_ack = False
                self._call_trigger(self.arm_client, "arm", required=True)
            elif action.kind is ActionKind.LAND:
                if self._landing_target is None or self.mission.state is State.ABORTED:
                    self._request_land()
            elif action.kind in (ActionKind.CANCEL_PLANNING, ActionKind.HOLD):
                self._hold()

    def _publish_path(self, action: Action) -> None:
        message = TrajectoryPath()
        message.stop_on_waypoints = action.kind is ActionKind.FLY_LANDING
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

    def _call_trigger(self, client, label: str, required: bool = False) -> None:
        if not client.service_is_ready():
            self.get_logger().warning(f"{label}: service not ready, request dropped")
            if required and label in ("arm", "takeoff"):
                self._fail_start(f"{label}: service not ready")
            return

        def _done(future) -> None:
            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001 - surfaced in the log
                self.get_logger().error(f"{label} failed: {exc}")
                if required and label in ("arm", "takeoff"):
                    self._fail_start(f"{label} failed: {exc}")
                return
            # Humble caches severity per call site; a dynamically selected log
            # method at one line raises if a failed response precedes success.
            if result.success:
                self.get_logger().info(f"{label}: success=True message={result.message!r}")
            else:
                self.get_logger().warning(f"{label}: success=False message={result.message!r}")
            if label == "arm" and self.mission.state is State.TAKEOFF:
                self._arm_ack = bool(result.success)
            if not result.success and required and label in ("arm", "takeoff"):
                self._fail_start(f"{label} rejected: {result.message}")

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
            detection=dict(self._last_detection),
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
