"""Monochrome frame baseline; it cannot identify a gate or measure depth.

Require a nested quadrilateral opening so a filled obstacle box is less likely
to be mistaken for a frame. Real camera validation is still required.
"""
from __future__ import annotations

from .color_gate import DetectorSettings, find_quads
from .common import PixelCandidate, centroid, order_quad, quad_area


class GrayscaleGateDetector:
    name = "grayscale"

    def __init__(self, settings: DetectorSettings | None = None) -> None:
        self.settings = settings or DetectorSettings()

    def detect(self, frame):
        import cv2
        if frame is None or getattr(frame, "size", 0) == 0:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        edges = cv2.Canny(gray, 40, 120)
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return []
        image_area = float(gray.shape[0]*gray.shape[1])
        candidates = []
        for index, contour in enumerate(contours):
            child = int(hierarchy[0][index][2])
            if child < 0:
                continue
            outer_area = float(cv2.contourArea(contour))
            # Canny yields two almost coincident edges around each boundary.
            # Find a genuinely smaller enclosed quadrilateral, not that twin.
            nested = []
            stack = [child]
            while stack:
                current = stack.pop()
                ratio = cv2.contourArea(contours[current])/max(outer_area, 1)
                if .25 < ratio < .85:
                    nested.extend(find_quads([contours[current]], self.settings, cv2, image_area))
                sibling = int(hierarchy[0][current][0])
                descendant = int(hierarchy[0][current][2])
                if sibling >= 0:
                    stack.append(sibling)
                if descendant >= 0:
                    stack.append(descendant)
            if not nested or not find_quads([contour], self.settings, cv2, image_area):
                continue
            opening = max(nested, key=quad_area)
            ordered = order_quad(opening)
            u, v = centroid(ordered)
            if any((candidate.centre_u-u)**2+(candidate.centre_v-v)**2 < 25 for candidate in candidates):
                continue
            candidates.append(PixelCandidate(centre_u=u, centre_v=v, area_px=quad_area(ordered),
                                             quad=tuple(ordered), confidence=.5))
        return candidates
