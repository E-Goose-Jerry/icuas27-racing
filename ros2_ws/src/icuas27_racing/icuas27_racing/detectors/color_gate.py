"""Colour-threshold gate detector.

This is the cheap, dependency-light baseline: threshold the gate frame colour in
HSV, take the largest quadrilateral contour, and report its centre. It is the
detector to get the pipeline working with; a learned detector can replace it
behind the same :class:`GateDetector` protocol without touching the node.

Known limits, stated so they are not mistaken for guarantees:

* The colour range depends on the gate material and the lighting of the arena.
  It is a parameter, not a constant, and it has to be tuned against real frames.
* A quadrilateral contour is assumed to be the gate opening. Cluttered
  backgrounds with square objects in the same colour will win the area contest.
* Range is not measured; only the bearing and the pixel area are. Use the
  course geometry for depth, or add a depth sensor.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable, Protocol

from .common import PixelCandidate, centroid, order_quad, quad_area, select_best

__all__ = ["ColorGateDetector", "DetectorSettings", "GateDetector", "find_quads"]


@dataclass(frozen=True, slots=True)
class DetectorSettings:
    """Everything tunable about the baseline detector."""

    hsv_lower: tuple[int, int, int] = (0, 60, 60)
    hsv_upper: tuple[int, int, int] = (180, 255, 255)
    min_area_px: float = 400.0
    max_area_fraction: float = 0.6
    morphology_kernel: int = 5
    approx_epsilon_fraction: float = 0.04
    expected_aspect: float = 1.0
    aspect_tolerance: float = 0.45

    def to_dict(self) -> dict[str, Any]:
        return {
            "hsv_lower": list(self.hsv_lower),
            "hsv_upper": list(self.hsv_upper),
            "min_area_px": self.min_area_px,
            "max_area_fraction": self.max_area_fraction,
            "morphology_kernel": self.morphology_kernel,
            "approx_epsilon_fraction": self.approx_epsilon_fraction,
            "expected_aspect": self.expected_aspect,
            "aspect_tolerance": self.aspect_tolerance,
        }


class GateDetector(Protocol):
    """Minimal contract every detector backend has to satisfy."""

    name: str

    def detect(self, frame: Any) -> list[PixelCandidate]:  # pragma: no cover - protocol
        ...


def _aspect_ok(width: float, height: float, settings: DetectorSettings) -> bool:
    if height <= 0.0 or width <= 0.0:
        return False
    aspect = width / height
    return abs(aspect - settings.expected_aspect) <= settings.aspect_tolerance


def find_quads(contours: Iterable[Any], settings: DetectorSettings, cv2: Any, image_area: float) -> list[Any]:
    """Filter contours down to gate-shaped quadrangles."""
    quads = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < settings.min_area_px:
            continue
        if image_area > 0.0 and area / image_area > settings.max_area_fraction:
            continue

        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 0.0:
            continue
        approximation = cv2.approxPolyDP(
            contour, settings.approx_epsilon_fraction * perimeter, True
        )
        if len(approximation) != 4 or not cv2.isContourConvex(approximation):
            continue

        points = [(float(point[0][0]), float(point[0][1])) for point in approximation]
        _, _, width, height = cv2.boundingRect(approximation)
        if not _aspect_ok(float(width), float(height), settings):
            continue
        quads.append(points)
    return quads


class ColorGateDetector:
    """HSV threshold + quadrangle contour detector."""

    name = "color"

    def __init__(self, settings: DetectorSettings | None = None) -> None:
        try:
            import cv2  # noqa: F401 - imported lazily so tests need no OpenCV
        except ImportError as exc:  # pragma: no cover - environment issue
            raise RuntimeError(
                "the colour detector needs OpenCV: sudo apt install python3-opencv"
            ) from exc
        self.settings = settings or DetectorSettings()

    def detect(self, frame: Any) -> list[PixelCandidate]:
        import cv2

        if frame is None or getattr(frame, "size", 0) == 0:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = tuple(int(v) for v in self.settings.hsv_lower)
        upper = tuple(int(v) for v in self.settings.hsv_upper)
        mask = cv2.inRange(hsv, lower, upper)

        kernel_size = max(1, int(self.settings.morphology_kernel))
        if kernel_size > 1:
            kernel = cv2.getStructuringElement(
                cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        height, width = frame.shape[:2]
        quads = find_quads(contours, self.settings, cv2, float(height * width))

        candidates: list[PixelCandidate] = []
        for points in quads:
            ordered = order_quad(points)
            centre_u, centre_v = centroid(ordered)
            area = quad_area(ordered)
            # A quad that fills a large share of the frame is more trustworthy
            # than a distant speck, but only up to a point.
            confidence = min(1.0, 0.3 + area / max(1.0, float(height * width)) * 6.0)
            candidates.append(
                PixelCandidate(
                    centre_u=centre_u,
                    centre_v=centre_v,
                    area_px=area,
                    quad=tuple(ordered),
                    confidence=confidence,
                )
            )
        return candidates

    def best(self, frame: Any, preferred_u: float | None = None) -> PixelCandidate | None:
        return select_best(self.detect(frame), preferred_u)
