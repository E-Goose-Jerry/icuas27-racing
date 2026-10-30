"""Safety gate: stale data budgets, bounds and altitude."""

from __future__ import annotations

import unittest

from racing_core.geometry import Vec3
from racing_core.safety import (
    ALTITUDE_HIGH,
    ALTITUDE_LOW,
    FCU_STALE,
    OUT_OF_BOUNDS,
    POSE_INVALID,
    POSE_STALE,
    Level,
    SafetyLimits,
    SafetyMonitor,
    distance_ok,
)


class PlatformBudgetTest(unittest.TestCase):
    def test_sim_gets_the_wider_budget(self) -> None:
        limits = SafetyLimits()
        self.assertAlmostEqual(limits.pose_timeout("sim"), limits.pose_timeout_sec_sim)
        self.assertAlmostEqual(limits.pose_timeout("real"), limits.pose_timeout_sec_real)
        self.assertGreater(limits.pose_timeout("sim"), limits.pose_timeout("real"))
        self.assertGreater(limits.fcu_timeout("sim"), limits.fcu_timeout("real"))

    def test_unknown_platform_falls_back_to_real(self) -> None:
        limits = SafetyLimits()
        self.assertAlmostEqual(limits.pose_timeout("banana"), limits.pose_timeout_sec_real)
        self.assertAlmostEqual(limits.pose_timeout(None), limits.pose_timeout_sec_real)

    def test_monitor_normalises_platform(self) -> None:
        self.assertEqual(SafetyMonitor(platform="SIM").platform, "sim")
        self.assertEqual(SafetyMonitor(platform=" Real ").platform, "real")

    def test_limits_roundtrip(self) -> None:
        limits = SafetyLimits(max_altitude=20.0, bounds_xy=(-1.0, 1.0, -2.0, 2.0))
        again = SafetyLimits.from_dict(limits.to_dict())
        self.assertEqual(again, limits)

    def test_limits_from_dict_rejects_bad_bounds(self) -> None:
        with self.assertRaises(ValueError):
            SafetyLimits.from_dict({"bounds_xy": [0.0, 1.0]})


class VerdictTest(unittest.TestCase):
    def setUp(self) -> None:
        self.monitor = SafetyMonitor(platform="sim")

    def test_healthy_sample_is_ok(self) -> None:
        verdict = self.monitor.check(10.0, pose_at=9.9, pose=Vec3(0.0, 0.0, 1.0))
        self.assertIs(verdict.level, Level.OK)
        self.assertTrue(verdict.ok)
        self.assertEqual(verdict.reasons, ())

    def test_no_pose_on_ground_is_a_warning(self) -> None:
        verdict = self.monitor.check(10.0, pose_at=None, pose=None)
        self.assertIs(verdict.level, Level.WARN)
        self.assertIn(POSE_STALE, verdict.reasons)

    def test_no_pose_while_airborne_aborts(self) -> None:
        verdict = self.monitor.check(10.0, pose_at=None, pose=None, airborne=True)
        self.assertIs(verdict.level, Level.ABORT)
        self.assertTrue(verdict.must_abort)

    def test_stale_pose_aborts_when_airborne(self) -> None:
        verdict = self.monitor.check(20.0, pose_at=10.0, pose=Vec3(0, 0, 1.0), airborne=True)
        self.assertIs(verdict.level, Level.ABORT)
        self.assertIn(POSE_STALE, verdict.reasons)
        self.assertAlmostEqual(verdict.pose_age, 10.0)

    def test_stale_pose_warns_on_the_ground(self) -> None:
        verdict = self.monitor.check(20.0, pose_at=10.0, pose=Vec3(0, 0, 0.0), airborne=False)
        self.assertIs(verdict.level, Level.WARN)

    def test_real_platform_is_stricter(self) -> None:
        real = SafetyMonitor(platform="real")
        sim = SafetyMonitor(platform="sim")
        # 0.6 s old: fine in sim, stale on real hardware
        self.assertIs(sim.check(10.6, pose_at=10.0, pose=Vec3(0, 0, 1.0)).level, Level.OK)
        self.assertIs(real.check(10.6, pose_at=10.0, pose=Vec3(0, 0, 1.0)).level, Level.WARN)

    def test_non_finite_pose_aborts(self) -> None:
        verdict = self.monitor.check(10.0, pose_at=10.0, pose=Vec3(float("nan"), 0, 1))
        self.assertIs(verdict.level, Level.ABORT)
        self.assertIn(POSE_INVALID, verdict.reasons)

    def test_out_of_bounds_aborts(self) -> None:
        verdict = self.monitor.check(10.0, pose_at=10.0, pose=Vec3(99.0, 0.0, 1.0))
        self.assertIs(verdict.level, Level.ABORT)
        self.assertIn(OUT_OF_BOUNDS, verdict.reasons)

    def test_too_high_aborts(self) -> None:
        verdict = self.monitor.check(10.0, pose_at=10.0, pose=Vec3(0.0, 0.0, 50.0))
        self.assertIn(ALTITUDE_HIGH, verdict.reasons)
        self.assertIs(verdict.level, Level.ABORT)

    def test_too_low_warns(self) -> None:
        verdict = self.monitor.check(10.0, pose_at=10.0, pose=Vec3(0.0, 0.0, 0.05))
        self.assertIn(ALTITUDE_LOW, verdict.reasons)
        self.assertIs(verdict.level, Level.WARN)

    def test_fcu_required_but_missing_aborts_when_airborne(self) -> None:
        verdict = self.monitor.check(
            10.0,
            pose_at=10.0,
            pose=Vec3(0, 0, 1),
            fcu_at=None,
            require_fcu=True,
            airborne=True,
        )
        self.assertIn(FCU_STALE, verdict.reasons)
        self.assertIs(verdict.level, Level.ABORT)

    def test_fcu_stale_aborts(self) -> None:
        verdict = self.monitor.check(
            30.0, pose_at=29.9, pose=Vec3(0, 0, 1), fcu_at=10.0, require_fcu=True
        )
        self.assertIn(FCU_STALE, verdict.reasons)
        self.assertIs(verdict.level, Level.ABORT)

    def test_multiple_reasons_are_deduplicated_and_keep_the_worst_level(self) -> None:
        monitor = SafetyMonitor(
            limits=SafetyLimits(bounds_xy=(-1.0, 1.0, -1.0, 1.0), min_altitude=0.5),
            platform="real",
        )
        verdict = monitor.check(20.0, pose_at=10.0, pose=Vec3(5.0, 0.0, 0.1))
        self.assertIs(verdict.level, Level.ABORT)
        self.assertEqual(len(verdict.reasons), len(set(verdict.reasons)))

    def test_last_verdict_is_remembered(self) -> None:
        self.assertIs(self.monitor.last.level, Level.OK)
        self.monitor.check(10.0, pose_at=None, pose=None)
        self.assertIs(self.monitor.last.level, Level.WARN)
        self.assertEqual(self.monitor.to_dict()["last"]["level"], "WARN")

    def test_verdict_serialises(self) -> None:
        verdict = self.monitor.check(10.0, pose_at=None, pose=None)
        payload = verdict.to_dict()
        self.assertEqual(payload["level"], "WARN")
        self.assertIsInstance(payload["reasons"], list)


class DistanceTest(unittest.TestCase):
    def test_within_tolerance(self) -> None:
        self.assertTrue(distance_ok(Vec3(0, 0, 1), Vec3(0.1, 0, 1), 0.2))
        self.assertFalse(distance_ok(Vec3(0, 0, 1), Vec3(0.3, 0, 1), 0.2))

    def test_bad_tolerance(self) -> None:
        self.assertFalse(distance_ok(Vec3(), Vec3(), -1.0))
        self.assertFalse(distance_ok(Vec3(), Vec3(), float("nan")))


if __name__ == "__main__":
    unittest.main()
