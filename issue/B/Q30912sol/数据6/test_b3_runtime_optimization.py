import importlib.util
import math
import sys
import unittest
from pathlib import Path


TARGET = Path(__file__).with_name("B3_robot_solver_v6_reviewable.py")
SPEC = importlib.util.spec_from_file_location("q3_runtime_target", TARGET)
assert SPEC is not None and SPEC.loader is not None
q3 = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = q3
SPEC.loader.exec_module(q3)


class RuntimeOptimizationTest(unittest.TestCase):
    class DummyApi:
        position = (0.0, 0.0)
        current_channel = 1
        virtual_time_s = 0.0

        def __init__(self) -> None:
            self.messages = []

    def test_solver_keeps_safe_layout_when_compact_cost_is_not_better(self) -> None:
        solver = q3.B3Solver(self.DummyApi())
        solver.select_search_layout()
        self.assertEqual(solver.layout_name, 'seven-point-safe')
        self.assertEqual(len(solver.ring_points), q3.SURVEY_N)

    def test_ring_tour_supports_six_point_candidate(self) -> None:
        solver = q3.B3Solver(self.DummyApi())
        solver.ring_points = q3.build_survey_points(6, 1135.0)[1:]
        solver.remaining_anchors = set(range(6))
        self.assertEqual(solver.choose_ring_tour(), list(range(6)))

    def test_compact_layout_is_certified_before_selection(self) -> None:
        self.assertTrue(q3.layout_is_certified(q3.build_survey_points(6, 1135.0)))
        self.assertFalse(q3.should_use_compact_layout(extra_move_s=120.0, unknown_n=12))

    def test_minimum_enclosing_circle_contains_vertices(self) -> None:
        center, radius = q3.minimum_enclosing_circle(
            [(0.0, 0.0), (4.0, 0.0), (0.0, 3.0)]
        )
        self.assertTrue(math.isclose(center[0], 2.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(center[1], 1.5, abs_tol=1e-9))
        self.assertTrue(math.isclose(radius, 2.5, abs_tol=1e-9))

    def test_minimum_enclosing_circle_handles_point_segment_and_collinear(self) -> None:
        center, radius = q3.minimum_enclosing_circle([(3.0, -2.0)])
        self.assertEqual(center, (3.0, -2.0))
        self.assertEqual(radius, 0.0)
        center, radius = q3.minimum_enclosing_circle([(0.0, 0.0), (6.0, 0.0)])
        self.assertTrue(math.isclose(center[0], 3.0, abs_tol=1e-9))
        self.assertTrue(math.isclose(radius, 3.0, abs_tol=1e-9))
        center, radius = q3.minimum_enclosing_circle(
            [(-2.0, 0.0), (0.0, 0.0), (5.0, 0.0)]
        )
        self.assertTrue(math.isclose(center[0], 1.5, abs_tol=1e-9))
        self.assertTrue(math.isclose(radius, 3.5, abs_tol=1e-9))

    def test_polygon_bound_remains_a_valid_vertex_bound(self) -> None:
        poly = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]
        center, radius = q3.polygon_bound(poly)
        self.assertTrue(all(q3.distance(center, p) <= radius + 1e-7 for p in poly))

    def test_compact_path_is_longer_than_safe_path(self) -> None:
        safe = q3.survey_path_length(q3.SURVEY_N, q3.SURVEY_R)
        compact = q3.survey_path_length(q3.COMPACT_SURVEY_N, q3.COMPACT_SURVEY_R)
        self.assertGreater(compact, safe)



if __name__ == "__main__":
    unittest.main()
