"""离线验证问题三的保证型扫描几何与有限清除兜底候选。"""

from __future__ import annotations

import math

import pytest

from src.problem_3.strategy import (
    clearance_grid,
    guaranteed_second_point,
    initial_scan_points,
    worst_initial_scan_distance,
)


def test_initial_scan_points_are_center_and_counterclockwise_hexagon() -> None:
    """首次扫描固定从 0° 开始逆时针给出圆心与六个正六边形顶点。"""

    points = initial_scan_points()

    assert len(points) == 7
    assert points[0] == (0.0, 0.0)
    expected_vertices = (
        (1200.0, 0.0),
        (600.0, 600.0 * math.sqrt(3.0)),
        (-600.0, 600.0 * math.sqrt(3.0)),
        (-1200.0, 0.0),
        (-600.0, -600.0 * math.sqrt(3.0)),
        (600.0, -600.0 * math.sqrt(3.0)),
    )
    for actual, expected in zip(points[1:], expected_vertices):
        assert actual == pytest.approx(expected)


@pytest.mark.parametrize("radius_m", [0.0, -0.1, math.inf, -math.inf, math.nan, True])
def test_initial_scan_points_reject_non_positive_or_non_finite_radius(radius_m: float) -> None:
    """首次扫描半径必须是正的有限实数。"""

    with pytest.raises(ValueError):
        initial_scan_points(radius_m)


def test_worst_initial_scan_distance_uses_stated_hexagon_bound() -> None:
    """首次扫描的覆盖上界应直接等于题设给出的余弦公式。"""

    distance = worst_initial_scan_distance()
    expected = math.sqrt(
        1800.0**2 + 1200.0**2 - 2.0 * 1800.0 * 1200.0 * math.cos(math.pi / 6.0)
    )

    assert distance == pytest.approx(expected)
    assert distance == pytest.approx(968.9016, abs=1e-4)
    assert distance < 1000.0


@pytest.mark.parametrize(
    ("target_radius_m", "scan_radius_m"),
    [
        (0.0, 1200.0),
        (1800.0, 0.0),
        (math.nan, 1200.0),
        (1800.0, math.inf),
        (True, 1200.0),
    ],
)
def test_worst_initial_scan_distance_rejects_invalid_radii(
    target_radius_m: float, scan_radius_m: float
) -> None:
    """两个半径都必须是正的有限实数。"""

    with pytest.raises(ValueError):
        worst_initial_scan_distance(target_radius_m, scan_radius_m)


def test_guaranteed_second_point_uses_forward_and_lateral_offsets() -> None:
    """0° 测向的默认二次点应为前向 750m、左侧 600m。"""

    assert guaranteed_second_point((0.0, 0.0), 0.0) == pytest.approx((750.0, 600.0))
    assert guaranteed_second_point((0.0, 0.0), 0.0, -1.0) == pytest.approx((750.0, -600.0))


def test_guaranteed_second_point_keeps_stated_worst_case_receivable() -> None:
    """距首点 1500m、相对测向 -1° 的题面最坏例到二次点仍小于 1000m。"""

    target = (1500.0 * math.cos(math.radians(-1.0)), 1500.0 * math.sin(math.radians(-1.0)))
    second = guaranteed_second_point((0.0, 0.0), 0.0)

    assert math.dist(target, second) < 1000.0


@pytest.mark.parametrize(
    ("first", "bearing_deg", "lateral_sign"),
    [
        ((math.inf, 0.0), 0.0, 1.0),
        ((0.0,), 0.0, 1.0),
        ((0.0, 0.0), math.nan, 1.0),
        ((0.0, 0.0), math.inf, 1.0),
        ((0.0, 0.0), 0.0, 0.0),
        ((0.0, 0.0), 0.0, 2.0),
        ((0.0, 0.0), 0.0, True),
    ],
)
def test_guaranteed_second_point_rejects_invalid_geometry_inputs(
    first: tuple[float, ...], bearing_deg: float, lateral_sign: float
) -> None:
    """测向、首点坐标与横向符号不合法时必须明确失败。"""

    with pytest.raises(ValueError):
        guaranteed_second_point(first, bearing_deg, lateral_sign)


def _nearest_distance(point: tuple[float, float], candidates: tuple[tuple[float, float], ...]) -> float:
    """返回一个待清除位置到候选集的最近距离。"""

    return min(math.dist(point, candidate) for candidate in candidates)


def test_clearance_grid_covers_non_integral_rectangle_samples() -> None:
    """端点补齐格点必须覆盖非整步长矩形的角点、中心和若干内部采样点。"""

    bounds = (-13.7, 4.2, 47.9, 83.6)
    radius_m = 20.0
    candidates = clearance_grid(bounds, radius_m)
    xmin, ymin, xmax, ymax = bounds
    samples = (
        (xmin, ymin),
        (xmin, ymax),
        (xmax, ymin),
        (xmax, ymax),
        ((xmin + xmax) / 2.0, (ymin + ymax) / 2.0),
        (0.0, 20.0),
        (33.3, 75.0),
    )

    assert candidates
    assert all(_nearest_distance(point, candidates) <= radius_m + 1e-9 for point in samples)


def test_clearance_grid_has_no_axis_gap_larger_than_one_step() -> None:
    """相邻格点及边界端点的间隔都不超过一个轴向步长。"""

    bounds = (-13.7, 4.2, 47.9, 83.6)
    radius_m = 20.0
    step = radius_m * math.sqrt(2.0)
    candidates = clearance_grid(bounds, radius_m)
    x_values = sorted({point[0] for point in candidates})
    y_values = sorted({point[1] for point in candidates})

    assert x_values[0] == pytest.approx(bounds[0])
    assert x_values[-1] == pytest.approx(bounds[2])
    assert y_values[0] == pytest.approx(bounds[1])
    assert y_values[-1] == pytest.approx(bounds[3])
    assert all(right - left <= step + 1e-9 for left, right in zip(x_values, x_values[1:]))
    assert all(right - left <= step + 1e-9 for left, right in zip(y_values, y_values[1:]))


@pytest.mark.parametrize(
    ("bounds", "samples"),
    [
        ((5.0, -10.0, 5.0, 40.0), ((5.0, -10.0), (5.0, 15.0), (5.0, 40.0))),
        ((2.0, 3.0, 2.0, 3.0), ((2.0, 3.0),)),
    ],
)
def test_clearance_grid_keeps_degenerate_bounds_coverable(
    bounds: tuple[float, float, float, float], samples: tuple[tuple[float, float], ...]
) -> None:
    """线和点边界也要得到有限、非空且可覆盖的清除候选。"""

    candidates = clearance_grid(bounds)

    assert candidates
    assert all(math.isfinite(value) for point in candidates for value in point)
    assert all(_nearest_distance(point, candidates) <= 20.0 + 1e-9 for point in samples)


@pytest.mark.parametrize(
    ("bounds", "radius_m"),
    [
        ((1.0, 0.0, 0.0, 1.0), 20.0),
        ((0.0, 1.0, 1.0, 0.0), 20.0),
        ((0.0, 0.0, math.inf, 1.0), 20.0),
        ((0.0, 0.0, 1.0), 20.0),
        ((0.0, 0.0, 1.0, 1.0), 0.0),
        ((0.0, 0.0, 1.0, 1.0), math.nan),
        ((0.0, 0.0, 1.0, 1.0), True),
    ],
)
def test_clearance_grid_rejects_invalid_bounds_or_radius(
    bounds: tuple[float, ...], radius_m: float
) -> None:
    """边界顺序、边界有限性与清除半径均须在生成前验证。"""

    with pytest.raises(ValueError):
        clearance_grid(bounds, radius_m)


def test_clearance_grid_documents_its_limited_fallback_role() -> None:
    """此网格明确只是有限清除兜底候选，不能被误读成已证高效策略。"""

    assert "清除兜底候选" in (clearance_grid.__doc__ or "")
