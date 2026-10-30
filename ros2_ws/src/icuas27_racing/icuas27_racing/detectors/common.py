"""Camera-model helpers shared by the detector backends.

Keeping the projection maths separate from OpenCV means the numbers that matter
(bearing in degrees, pixel-to-angle mapping) can be unit tested with plain
tuples, on a machine where ``cv2`` is not installed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

__all__ = [
    "Intrinsics",
    "PixelCandidate",
    "bearing",
    "centroid",
    "order_quad",
    "quad_area",
    "select_best",
]


@dataclass(frozen=True, slots=True)
class Intrinsics:
    """Pinhole camera model for one image size."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def __post_init__(self) -> None:
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("focal lengths must be positive")

    def horizontal_fov(self) -> float:
        return 2.0 * math.atan2(self.width / 2.0, self.fx)

    def vertical_fov(self) -> float:
        return 2.0 * math.atan2(self.height / 2.0, self.fy)

    def scaled(self, width: int, height: int) -> "Intrinsics":
        """Same lens, different image size (e.g. downsampled frames)."""
        if self.width <= 0 or self.height <= 0:
            raise ValueError("source image size must be positive")
        sx = width / float(self.width)
        sy = height / float(self.height)
        return Intrinsics(
            fx=self.fx * sx,
            fy=self.fy * sy,
            cx=self.cx * sx,
            cy=self.cy * sy,
            width=width,
            height=height,
        )


def bearing(intrinsics: Intrinsics, u: float, v: float) -> tuple[float, float]:
    """Pixel -> ``(yaw, pitch)`` in radians, optical axis pointing forward."""
    yaw = math.atan2(u - intrinsics.cx, intrinsics.fx)
    pitch = math.atan2(v - intrinsics.cy, intrinsics.fy)
    return yaw, pitch


@dataclass(frozen=True, slots=True)
class PixelCandidate:
    """A gate hypothesis in image space."""

    centre_u: float
    centre_v: float
    area_px: float
    quad: tuple[tuple[float, float], ...] = ()
    confidence: float = 1.0

    def to_dict(self) -> dict[str, object]:
        return {
            "centre_u": self.centre_u,
            "centre_v": self.centre_v,
            "area_px": self.area_px,
            "confidence": self.confidence,
            "quad": [list(point) for point in self.quad],
        }


def centroid(points: list[tuple[float, float]]) -> tuple[float, float]:
    if not points:
        raise ValueError("cannot take the centroid of no points")
    sx = sum(point[0] for point in points)
    sy = sum(point[1] for point in points)
    count = float(len(points))
    return sx / count, sy / count


def quad_area(points: list[tuple[float, float]]) -> float:
    """Shoelace area; works for convex polygons in any winding order."""
    if len(points) < 3:
        return 0.0
    total = 0.0
    count = len(points)
    for index in range(count):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % count]
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


def order_quad(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Order four points as top-left, top-right, bottom-right, bottom-left.

    Uses the image convention (v grows downwards), which is what OpenCV hands
    us. Fewer or more than four points are returned unchanged.
    """
    if len(points) != 4:
        return list(points)
    ordered = sorted(points, key=lambda point: (point[1], point[0]))  # by v then u
    top = sorted(ordered[:2], key=lambda point: point[0])
    bottom = sorted(ordered[2:], key=lambda point: point[0])
    return [top[0], top[1], bottom[1], bottom[0]]


def select_best(candidates: list[PixelCandidate], preferred_u: float | None = None) -> PixelCandidate | None:
    """Pick the most likely gate.

    Larger and more confident wins; when ``preferred_u`` is given, candidates
    near the image centre are preferred so the vehicle follows the gate it is
    actually heading for.
    """
    if not candidates:
        return None

    def score(candidate: PixelCandidate) -> float:
        value = candidate.area_px * max(0.0, candidate.confidence)
        if preferred_u is not None:
            value /= 1.0 + abs(candidate.centre_u - preferred_u) / 100.0
        return value

    return max(candidates, key=score)
