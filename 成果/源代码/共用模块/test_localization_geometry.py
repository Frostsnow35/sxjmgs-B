"""Regression tests for the offline geometry used by questions 1 and 2."""

from __future__ import annotations

import math

import pytest

from src.localization_geometry import (
    diameter_circle_covers,
    intersection_sine,
    locate_from_bearings,
    rank_second_points,
    sample_first_direction_region,
    sample_target_disk,
    strict_second_point_kernel,
    worst_case_intersection_sine,
)


def test_intersection_of_two_exact_bearings_is_a_point() -> None:
    """Two orthogonal exact rays must localize their unique intersection."""
    result = locate_from_bearings(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 90.0)),
        error_deg=0.0,
    )

    assert result.status == "point"
    assert result.diameter_m == pytest.approx(0.0)
    assert len(result.vertices) == 1
    assert result.vertices[0][0] == pytest.approx(1.0)
    assert result.vertices[0][1] == pytest.approx(0.0)


def test_parallel_exact_bearings_with_no_shared_point_are_empty() -> None:
    """Parallel but distinct exact rays have no common feasible source."""
    result = locate_from_bearings(
        ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        error_deg=0.0,
    )

    assert result.status == "empty"
    assert result.diameter_m is None
    assert result.diameter_circle_covers is None


def test_single_nonzero_error_bearing_is_unbounded() -> None:
    """One angular cone alone cannot have a finite localization diameter."""
    result = locate_from_bearings(
        ((0.0, 0.0, 359.0),),
        error_deg=1.0,
        target_radius_m=None,
    )

    assert result.status == "unbounded"
    assert result.diameter_m is None


def test_target_disk_clips_single_bearing_to_a_finite_region() -> None:
    """The stated circular target domain must bound a one-bearing region exactly."""
    result = locate_from_bearings(
        ((0.0, 0.0, 0.0),),
        error_deg=1.0,
        target_radius_m=100.0,
    )

    assert result.status == "disk_clipped"
    assert result.diameter_m == pytest.approx(100.0)
    assert all(math.hypot(*point) <= 100.0 + 1e-8 for point in result.vertices)
    assert result.diameter_circle_covers is False


def test_crossing_zero_degree_is_treated_continuously() -> None:
    """A direction interval around 0 degrees must include due-east points."""
    result = locate_from_bearings(
        ((0.0, 0.0, 359.0), (1.0, -1.0, 90.0)),
        error_deg=2.0,
    )

    assert result.status == "polygon"
    assert min(vertex[0] for vertex in result.vertices) < 1.0
    assert max(vertex[0] for vertex in result.vertices) > 1.0
    assert min(vertex[1] for vertex in result.vertices) < 0.0
    assert max(vertex[1] for vertex in result.vertices) > 0.0


def test_diameter_circle_does_not_cover_an_equilateral_triangle() -> None:
    """The equilateral triangle is a concrete counterexample to the claim."""
    root3 = math.sqrt(3.0)
    covers, diameter_m, centers = diameter_circle_covers(
        ((0.0, 0.0), (2.0, 0.0), (1.0, root3))
    )

    assert diameter_m == pytest.approx(2.0)
    assert covers is False
    assert len(centers) == 3


def test_perpendicular_second_point_has_better_worst_case_geometry() -> None:
    """Side-looking geometry must beat the collinear forward baseline."""
    source_points = ((100.0, 0.0), (200.0, 0.0))
    lateral_score = worst_case_intersection_sine(
        (0.0, 100.0), (0.0, 0.0), source_points
    )
    forward_score = worst_case_intersection_sine(
        (300.0, 0.0), (0.0, 0.0), source_points
    )

    assert lateral_score > forward_score
    assert forward_score == pytest.approx(0.0)


def test_intersection_sine_is_one_for_perpendicular_bearings() -> None:
    """The dimensionless conditioning score equals one at 90 degrees."""
    score = intersection_sine((0.0, 0.0), (1.0, -1.0), (1.0, 0.0))

    assert score == pytest.approx(1.0)


def test_ranked_candidates_prefer_higher_worst_case_intersection_angle() -> None:
    """Candidate ordering exposes a reproducible geometric baseline for Q2."""
    ranked = rank_second_points(
        first_point=(0.0, 0.0),
        source_points=((100.0, 0.0), (200.0, 0.0)),
        candidates=((300.0, 0.0), (0.0, 100.0)),
        guaranteed_receive_radius_m=1_000.0,
    )

    assert ranked[0].point == (0.0, 100.0)
    assert ranked[0].worst_case_sine > ranked[1].worst_case_sine
    assert all(item.guaranteed_receivable for item in ranked)


def test_first_direction_region_respects_disk_range_and_wrapped_bearing() -> None:
    """Question 2 sampling must retain only points consistent with the first read."""
    sampled = sample_first_direction_region(
        first_point=(0.0, 0.0),
        bearing_deg=359.0,
        grid_spacing_m=100.0,
        error_deg=2.0,
        target_radius_m=300.0,
        maximum_receive_radius_m=200.0,
    )

    assert (100.0, 0.0) in sampled
    assert all(math.dist(point, (0.0, 0.0)) <= 200.0 + 1e-8 for point in sampled)
    assert all(math.hypot(*point) <= 300.0 + 1e-8 for point in sampled)


def test_strict_second_point_kernel_enforces_angle_and_receive_guarantees() -> None:
    """The Q2 guarantee set excludes a collinear but receivable baseline point."""
    kernel = strict_second_point_kernel(
        first_point=(0.0, 0.0),
        source_points=((100.0, 0.0), (200.0, 0.0)),
        candidates=((300.0, 0.0), (0.0, 100.0)),
        angle_tolerance_deg=70.0,
        guaranteed_receive_radius_m=1_000.0,
    )

    assert tuple(item.point for item in kernel) == ((0.0, 100.0),)


def test_target_disk_sampler_includes_only_lattice_points_in_the_target_circle() -> None:
    """Candidate second points must remain inside the stated target region."""
    sampled = sample_target_disk(grid_spacing_m=100.0, target_radius_m=150.0)

    assert (0.0, 0.0) in sampled
    assert (100.0, 0.0) in sampled
    assert (200.0, 0.0) not in sampled
    assert all(math.hypot(*point) <= 150.0 + 1e-8 for point in sampled)
