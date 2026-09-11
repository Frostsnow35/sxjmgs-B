"""Regression tests for the Q4 location-orientation feasibility projector."""
from __future__ import annotations

import unittest
from types import SimpleNamespace
from unittest.mock import patch

import strategy

from joint_feasibility import (
    feasible_orientation,
    project_positions,
    safe_negative_points,
)


class JointFeasibilityTest(unittest.TestCase):
    """Tests for the pure, non-simulator feasibility predicate."""

    def test_rejects_location_requiring_radius_above_1500(self) -> None:
        result = feasible_orientation(
            (1600.0, 0.0), [((0.0, 0.0), 0.0)], []
        )
        self.assertFalse(result.feasible)
        self.assertGreater(result.r_min, 1500.0)

    def test_accepts_negative_explained_by_distance(self) -> None:
        result = feasible_orientation(
            (100.0, 0.0), [((0.0, 0.0), 0.0)], [(1400.0, 0.0)]
        )
        self.assertTrue(result.feasible)
        self.assertIsNotNone(result.witness_angle_deg)

    def test_reports_one_connected_orientation_arc(self) -> None:
        result = feasible_orientation(
            (100.0, 0.0), [((0.0, 0.0), 0.0)], []
        )
        self.assertEqual(result.arc_count, 1)

    def test_rejects_conflicting_in_range_negative(self) -> None:
        result = feasible_orientation(
            (100.0, 0.0), [((0.0, 0.0), 0.0)], [(50.0, 0.0)]
        )
        self.assertFalse(result.feasible)

    def test_convex_hull_negative_requires_distance_above_1000(self) -> None:
        signals = [
            ((-200.0, -100.0), 0.0),
            ((-100.0, 0.0), 0.0),
            ((-200.0, 100.0), 0.0),
        ]
        negatives = [(-160.0, 10.0)]
        self.assertEqual(safe_negative_points(signals, negatives), negatives)
        self.assertFalse(feasible_orientation((0.0, 0.0), signals, negatives).feasible)

    def test_projector_keeps_a_known_feasible_location(self) -> None:
        polygon = [(0.0, -50.0), (200.0, -50.0), (200.0, 50.0), (0.0, 50.0)]
        points = project_positions(
            polygon,
            [((0.0, 0.0), 0.0)],
            [(200.0, 200.0)],
            max_cell_m=50.0,
        )
        self.assertIn((100.0, 0.0), [point.position for point in points])

    def test_empty_joint_projection_preserves_conservative_cover(self) -> None:
        class FakeInterface:
            pos = (0.0, 0.0)

            def measure(self, point, channel):
                self.pos = point
                return SimpleNamespace(measure_result="no_signal")

            def clear(self, point, channel):
                self.pos = point
                return SimpleNamespace(clear_result="no_target_in_range")

        data = strategy.ChannelData(
            signals=[((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
        )
        with patch.object(strategy.joint, "project_positions", return_value=[]), \
             patch.object(strategy, "_cover_and_clear", return_value=True) as cover:
            self.assertTrue(strategy.clear_channel_q4(FakeInterface(), 1, data))
        self.assertTrue(cover.called)

    def test_projected_sample_does_not_trigger_preemptive_clear(self) -> None:
        class TrackingInterface:
            pos = (0.0, 0.0)

            def __init__(self) -> None:
                self.actions = []

            def measure(self, point, channel):
                self.pos = point
                self.actions.append(("measure", point))
                return SimpleNamespace(measure_result="no_signal")

            def clear(self, point, channel):
                self.pos = point
                self.actions.append(("clear", point))
                return SimpleNamespace(clear_result="no_target_in_range")

        iface = TrackingInterface()
        data = strategy.ChannelData(
            signals=[((0.0, 0.0), 0.0), ((100.0, 0.0), 0.0)]
        )
        with patch.object(strategy, "_joint_projection_summary", return_value=((100.0, 0.0), 0.0, 1)), \
             patch.object(strategy, "_cover_and_clear", return_value=True):
            self.assertTrue(strategy.clear_channel_q4(iface, 1, data))
        self.assertEqual(iface.actions[0][0], "measure")

    def test_cover_uses_joint_projection_only_to_prioritize_candidates(self) -> None:
        class ClearingInterface:
            pos = (0.0, 0.0)

            def __init__(self) -> None:
                self.attempts = []

            def clear(self, point, channel):
                self.attempts.append(point)
                return SimpleNamespace(clear_result="success")

        iface = ClearingInterface()
        with patch.object(strategy.geo, "cover_points_for_polygon", return_value=[(-30.0, 0.0), (30.0, 0.0)]):
            self.assertTrue(strategy._cover_and_clear(
                iface,
                [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
                1,
                priority_center=(30.0, 0.0),
            ))
        self.assertEqual(iface.attempts, [(30.0, 0.0)])


if __name__ == "__main__":
    unittest.main()
