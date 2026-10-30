"""Gate detector backends.

``common`` holds the projection maths and is ROS/OpenCV free; ``color_gate`` is
the baseline HSV + quadrangle detector.
"""

from .common import Intrinsics, PixelCandidate, bearing, centroid, order_quad, quad_area, select_best

__all__ = [
    "Intrinsics",
    "PixelCandidate",
    "bearing",
    "centroid",
    "order_quad",
    "quad_area",
    "select_best",
]
