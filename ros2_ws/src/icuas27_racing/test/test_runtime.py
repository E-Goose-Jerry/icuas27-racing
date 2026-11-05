"""ROS-backed adapter checks; no simulator or real vehicle is contacted."""
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.task import Future
from laser_msgs.msg import ApiPx4Diagnostics, UavControlDiagnostics
from nav_msgs.msg import Odometry
from icuas27_interfaces.srv import RaceControl
from icuas27_racing.race_node import RaceNode
from icuas27_racing.detectors.grayscale_gate import GrayscaleGateDetector
from racing_core.geometry import Vec3
from racing_core.mission_fsm import Action, ActionKind, State

ROOT = Path(__file__).resolve().parents[4]


class Client:
    def __init__(self, success=True):
        self.success = success
        self.calls = 0

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.calls += 1
        future = Future()
        future.set_result(SimpleNamespace(success=self.success, message="test response"))
        return future


class Publisher:
    def __init__(self):
        self.messages = []

    def get_subscription_count(self):
        return 1

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def node(tmp_path):
    rclpy.init(args=['--ros-args', '-p', f'course_file:={ROOT}/course/integration_smoke.json',
                     '-p', 'lus_backend:=control_manager', '-p', f'status_file:={tmp_path}/status.json'])
    node = RaceNode()
    node.arm_client = Client()
    node.takeoff_client = Client()
    node.land_client = Client()
    node.disarm_client = Client()
    node.path_pub = Publisher()
    node._on_fcu(ApiPx4Diagnostics(armed=False, offboard_mode=False, preflight_checks_passed=True))
    node._on_flight(UavControlDiagnostics(is_fly=False))
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    node._on_odom(message)
    yield node
    node.destroy_node()
    rclpy.shutdown()


def start(node):
    return node._on_control(RaceControl.Request(command='start'), RaceControl.Response())


def test_start_refuses_missing_pose(node):
    node._last_pose = None
    assert not start(node).accepted
    assert node.arm_client.calls == 0
    assert node.mission.state is State.IDLE


def test_arm_rejection_does_not_send_takeoff(node):
    node.arm_client.success = False
    start(node)
    node._tick()
    assert node.takeoff_client.calls == 0
    assert node.mission.state is State.ABORTED


def test_takeoff_waits_for_actual_arm_and_control_ready(node):
    assert start(node).accepted
    node._tick()
    assert node.takeoff_client.calls == 0
    node._on_fcu(ApiPx4Diagnostics(armed=True, offboard_mode=True, preflight_checks_passed=True))
    node._tick()
    assert node.takeoff_client.calls == 1
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    message.pose.pose.position.z = 1.5
    node._on_odom(message)
    node._tick()
    assert node.mission.state is State.TAKEOFF
    node._on_flight(UavControlDiagnostics(is_fly=True))
    node._tick()
    assert node.mission.state is State.RACE
    assert len(node.path_pub.messages[-1].waypoints) == 3


def test_lost_fcu_triggers_hold_even_without_new_odom(node):
    node._armed = node._control_flying = node._airborne = True
    node._last_pose = Vec3(0, 0, 1.5)
    node._last_fcu_at -= 10
    node._tick()
    assert node.mission.state is State.SAFE_STOP
    assert node.path_pub.messages[-1].stop_on_waypoints
    assert len(node.path_pub.messages[-1].waypoints) == 1


def test_landing_waits_for_arrival_then_stopped_flight_and_disarm(node):
    node._armed = node._control_flying = node._airborne = True
    node.mission.state = State.LANDING
    node._landing_target = Vec3(0, 0, 1.5)
    node._last_pose = Vec3(1, 0, 1.5)
    node._tick()
    assert node.land_client.calls == 0
    node._last_pose = Vec3(.05, 0, 1.5)
    node._speed = .3
    node._tick()
    assert node.land_client.calls == 0
    node._speed = .02
    node._tick()
    assert node.land_client.calls == 1
    assert node.disarm_client.calls == 0
    node._control_flying = node._airborne = False
    node._last_pose = Vec3(.1, 0, .05)
    node._tick()
    assert node.disarm_client.calls == 1
    assert node.mission.state is State.LANDING
    node._armed = False
    node._tick()
    assert node.mission.state is State.DONE


def test_reset_refuses_inflight(node):
    node._armed = True
    response = node._on_control(RaceControl.Request(command='reset'), RaceControl.Response())
    assert not response.accepted


def test_monochrome_frame_detects_opening_but_not_solid_box():
    cv2 = pytest.importorskip('cv2')
    np = pytest.importorskip('numpy')
    image = np.full((244, 324, 3), 220, dtype=np.uint8)
    cv2.rectangle(image, (90, 50), (230, 190), (20, 20, 20), 14)
    assert GrayscaleGateDetector().detect(image)
    cv2.rectangle(image, (90, 50), (230, 190), (20, 20, 20), -1)
    assert not GrayscaleGateDetector().detect(image)


def test_landing_rejects_stale_pose_and_nonfinite_speed(node):
    node.mission.state = State.LANDING
    node._landing_target = node._last_pose = Vec3(0, 0, 1.5)
    node._speed = float('nan')
    node._tick()
    assert node.land_client.calls == 0
    node._speed = 0.0
    node._last_pose_at -= .6
    node._tick()
    assert node.land_client.calls == 0


def test_only_landing_path_requests_stop_at_waypoints(node):
    for kind, should_stop in [(ActionKind.FLY_LANDING, True), (ActionKind.FLY_GATE, False)]:
        node._publish_path(Action(kind=kind, waypoints=(Vec3(0, 0, 1.5),), speed=.3))
        assert node.path_pub.messages[-1].stop_on_waypoints is should_stop
