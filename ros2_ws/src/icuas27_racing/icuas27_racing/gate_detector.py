"""Gate detector node.

Subscribes to the front camera, runs the configured detector backend, and
publishes one :class:`GateObservation` per visible gate. It never commands the
vehicle: the race node decides what to do with a detection.

Default topics follow the official LUS Crazyflie setup (AI-deck camera, mono8,
324x244 at 30 Hz, ``/<uav>/camera/ai_deck/image_raw``). Point ``image_topic`` at
a different camera when using one.
"""

from __future__ import annotations

import math
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from icuas27_interfaces.msg import GateObservation

from .detectors.common import Intrinsics, PixelCandidate, bearing, select_best
from .detectors.color_gate import ColorGateDetector, DetectorSettings


def _default_intrinsics(width: int, height: int) -> Intrinsics:
    """Rough pinhole model for the AI-deck camera.

    Replace it with the calibrated values from ``camera_info`` (or the
    ``camera_info`` parameters) before trusting ranges or angles.
    """
    return Intrinsics(
        fx=float(width) * 0.62,
        fy=float(width) * 0.62,
        cx=float(width) / 2.0,
        cy=float(height) / 2.0,
        width=width,
        height=height,
    )


class GateDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("gate_detector")

        self.uav_name = self.declare_parameter("uav_name", "uav1").value
        self.image_topic = str(
            self.declare_parameter(
                "image_topic", f"/{self.uav_name}/camera/ai_deck/image_raw"
            ).value
        )
        self.camera_info_topic = str(
            self.declare_parameter(
                "camera_info_topic", f"/{self.uav_name}/camera/ai_deck/camera_info"
            ).value
        )
        self.observation_topic = str(
            self.declare_parameter("observation_topic", "/icuas27/gate_observation").value
        )
        self.publish_debug_image = bool(
            self.declare_parameter("publish_debug_image", False).value
        )
        self.debug_topic = str(
            self.declare_parameter("debug_topic", "/icuas27/gate_debug/image").value
        )

        settings = DetectorSettings(
            hsv_lower=tuple(int(v) for v in self.declare_parameter("hsv_lower", [0, 60, 60]).value),
            hsv_upper=tuple(int(v) for v in self.declare_parameter("hsv_upper", [180, 255, 255]).value),
            min_area_px=float(self.declare_parameter("min_area_px", 400.0).value),
            expected_aspect=float(self.declare_parameter("expected_aspect", 1.0).value),
        )
        self.detector = ColorGateDetector(settings)
        self._intrinsics: Intrinsics | None = None
        self._frames = 0
        self._detections = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(Image, self.image_topic, self._on_image, sensor_qos)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, sensor_qos
        )
        self.observation_pub = self.create_publisher(GateObservation, self.observation_topic, 10)

        self._debug_image_pub = None
        if self.publish_debug_image:
            self._debug_image_pub = self.create_publisher(Image, self.debug_topic, 5)
        self._cv_bridge = None
        try:
            from cv_bridge import CvBridge

            self._cv_bridge = CvBridge()
        except ImportError:
            self.get_logger().warning(
                "cv_bridge is not installed: only bgr8/rgb8 images can be converted"
            )

        self.get_logger().info(
            f"gate_detector({self.detector.name}) listening on {self.image_topic}, "
            f"publishing {self.observation_topic}"
        )

    # -- callbacks -------------------------------------------------------
    def _on_camera_info(self, message: CameraInfo) -> None:
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self._intrinsics = Intrinsics(
                fx=float(message.k[0]),
                fy=float(message.k[4]),
                cx=float(message.k[2]),
                cy=float(message.k[5]),
                width=int(message.width),
                height=int(message.height),
            )
            self.get_logger().info(
                f"camera calibrated from camera_info: fx={self._intrinsics.fx:.1f} "
                f"fy={self._intrinsics.fy:.1f}"
            )

    def _on_image(self, message: Image) -> None:
        frame = self._to_bgr(message)
        if frame is None:
            return
        self._frames += 1

        intrinsics = self._intrinsics or _default_intrinsics(
            int(message.width), int(message.height)
        )
        if intrinsics.width != int(message.width) or intrinsics.height != int(message.height):
            intrinsics = intrinsics.scaled(int(message.width), int(message.height))

        candidates = self.detector.detect(frame)
        self._detections += len(candidates)
        if not candidates:
            self._publish_empty(message, intrinsics)
            return

        best = select_best(candidates, preferred_u=intrinsics.cx)
        if best is not None:
            self._publish_observation(message, intrinsics, best)

        if self._debug_image_pub is not None:
            self._publish_debug(message, frame, candidates)

    # -- helpers ---------------------------------------------------------
    def _to_bgr(self, message: Image) -> Any | None:
        import numpy as np

        try:
            if self._cv_bridge is not None:
                return self._cv_bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            if message.encoding in ("bgr8", "8UC3"):
                return np.frombuffer(message.data, dtype=np.uint8).reshape(
                    message.height, message.width, 3
                )
            if message.encoding in ("mono8", "rgb8", "8UC1"):
                dtype = np.uint8
                channels = 1 if message.encoding == "mono8" else 3
                array = np.frombuffer(message.data, dtype=dtype)
                if channels == 1:
                    array = array.reshape(message.height, message.width)
                    return np.repeat(array[:, :, None], 3, axis=2)
                array = array.reshape(message.height, message.width, 3)
                return array[:, :, ::-1]
            self.get_logger().warning(f"unsupported image encoding: {message.encoding}")
            return None
        except Exception as exc:  # noqa: BLE001 - a bad frame must not kill the node
            self.get_logger().warning(f"cannot convert image: {exc}")
            return None

    def _publish_observation(
        self, message: Image, intrinsics: Intrinsics, candidate: PixelCandidate
    ) -> None:
        yaw, pitch = bearing(intrinsics, candidate.centre_u, candidate.centre_v)
        observation = GateObservation()
        observation.header = message.header
        observation.gate_id = 0
        observation.confidence = float(candidate.confidence)
        observation.visible = True
        # Image-space only: the node has no depth. ``position`` stays zero and
        # the mission uses the course geometry for the target.
        observation.position.x = 0.0
        observation.position.y = 0.0
        observation.position.z = 0.0
        observation.distance = -1.0
        observation.bearing_yaw = float(yaw)
        observation.bearing_pitch = float(pitch)
        observation.source = self.detector.name
        self.observation_pub.publish(observation)

    def _publish_empty(self, message: Image, intrinsics: Intrinsics) -> None:
        observation = GateObservation()
        observation.header = message.header
        observation.gate_id = 0
        observation.confidence = 0.0
        observation.visible = False
        observation.distance = -1.0
        observation.bearing_yaw = float("nan")
        observation.bearing_pitch = float("nan")
        observation.source = self.detector.name
        self.observation_pub.publish(observation)

    def _publish_debug(self, message: Image, frame: Any, candidates: list[PixelCandidate]) -> None:
        import cv2

        annotated = frame.copy()
        for candidate in candidates:
            if len(candidate.quad) == 4:
                points = [[int(u), int(v)] for u, v in candidate.quad]
                for index in range(4):
                    cv2.line(
                        annotated,
                        tuple(points[index]),
                        tuple(points[(index + 1) % 4]),
                        (0, 255, 0),
                        2,
                    )
            cv2.circle(
                annotated,
                (int(candidate.centre_u), int(candidate.centre_v)),
                4,
                (0, 0, 255),
                -1,
            )
        if self._cv_bridge is None:
            return
        try:
            out = self._cv_bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"cannot build debug image: {exc}")
            return
        out.header = message.header
        self._debug_image_pub.publish(out)

    # -- diagnostics -----------------------------------------------------
    def stats(self) -> dict[str, Any]:
        return {
            "frames": self._frames,
            "detections": self._detections,
            "hit_rate": (self._detections / self._frames) if self._frames else math.nan,
            "settings": self.detector.settings.to_dict(),
        }


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node: GateDetectorNode | None = None
    try:
        node = GateDetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
