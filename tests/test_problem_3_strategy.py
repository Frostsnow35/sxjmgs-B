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


def test_public_geometry_functions_normalize_huge_integer_overflow_to_value_error() -> None:
    """超大整数在半径和边界路径中均应统一为 ValueError，而非泄漏转换溢出。"""

    huge_integer = 10**1000

    with pytest.raises(ValueError):
        initial_scan_points(huge_integer)
    with pytest.raises(ValueError):
        clearance_grid((0.0, 0.0, huge_integer, 1.0))


def test_worst_initial_scan_distance_uses_stated_hexagon_bound() -> None:
    """首次扫描的覆盖上界应直接等于题设给出的余弦公式。"""

    distance = worst_initial_scan_distance()
    expected = math.sqrt(
        1800.0**2 + 1200.0**2 - 2.0 * 1800.0 * 1200.0 * math.cos(math.pi / 6.0)
    )

    assert distance == pytest.approx(expected)
    assert distance == pytest.approx(968.9016, abs=1e-4)
    assert distance < 1000.0


def test_worst_initial_scan_distance_stays_finite_for_equal_huge_radii() -> None:
    """等大的极大有限半径不应因 inf-inf 消去而产生 NaN。"""

    assert math.isfinite(worst_initial_scan_distance(1e308, 1e308))


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
    "bounds",
    [
        (-13.7, 4.2, 47.9, 83.6),
        (float(2**56), float(2**56), float(2**56) + 32.0, float(2**56) + 32.0),
        (5.0, -10.0, 5.0, 40.0),
        (2.0, 3.0, 2.0, 3.0),
    ],
)
def test_clearance_grid_axis_certificate_uses_actual_float_gaps(
    bounds: tuple[float, float, float, float]
) -> None:
    """多类边界的实际浮点轴间隔应给出二维半径覆盖证书。"""

    radius_m = 20.0
    step = radius_m * math.sqrt(2.0)
    candidates = clearance_grid(bounds, radius_m)
    x_values = sorted({point[0] for point in candidates})
    y_values = sorted({point[1] for point in candidates})

    assert x_values[0] == bounds[0]
    assert x_values[-1] == bounds[2]
    assert y_values[0] == bounds[1]
    assert y_values[-1] == bounds[3]
    assert len(candidates) == len(x_values) * len(y_values)
    assert all(right - left <= step for left, right in zip(x_values, x_values[1:]))
    assert all(right - left <= step for left, right in zip(y_values, y_values[1:]))

    x_cell_centers = [
        left + (right - left) / 2.0 for left, right in zip(x_values, x_values[1:])
    ] or [x_values[0]]
    y_cell_centers = [
        left + (right - left) / 2.0 for left, right in zip(y_values, y_values[1:])
    ] or [y_values[0]]
    assert all(
        _nearest_distance((x, y), candidates) <= radius_m
        for x in x_cell_centers
        for y in y_cell_centers
    )


def test_clearance_grid_refines_multiple_large_coordinate_gaps_in_order() -> None:
    """大坐标下多个 32m 实际 gap 经分段扫描后应保留 9×9 个有序候选。"""

    base = float(2**56)
    radius_m = 20.0
    step = radius_m * math.sqrt(2.0)
    candidates = clearance_grid((base, base, base + 128.0, base + 128.0), radius_m)
    x_values = sorted({point[0] for point in candidates})
    y_values = sorted({point[1] for point in candidates})

    assert len(candidates) == 81
    assert len(x_values) == len(y_values) == 9
    assert [right - left for left, right in zip(x_values, x_values[1:])] == [16.0] * 8
    assert [right - left for left, right in zip(y_values, y_values[1:])] == [16.0] * 8
    assert all(right - left <= step for left, right in zip(x_values, x_values[1:]))
    assert all(right - left <= step for left, right in zip(y_values, y_values[1:]))


def test_clearance_grid_rejects_interval_without_representable_dense_points() -> None:
    """当 ULP 大于所需步长且没有严格中点时，不能虚假宣称 20m 覆盖。"""

    base = float(2**57)
    with pytest.raises(ValueError, match="浮点"):
        clearance_grid((base, base, base + 32.0, base + 32.0), 20.0)


def test_clearance_grid_rejects_candidate_count_over_its_documented_limit() -> None:
    """病态的大清除框超过显式候选上限时应快速拒绝，不能耗尽资源。"""

    with pytest.raises(ValueError, match="候选"):
        clearance_grid((0.0, 0.0, 10_000.0, 10_000.0), 20.0)


def test_clearance_grid_rejects_finite_bounds_with_overflowing_span_by_limit() -> None:
    """有限端点相减溢出时也要走候选上限的 ValueError 路径。"""

    with pytest.raises(ValueError, match="候选|上限"):
        clearance_grid((-1e308, 0.0, 1e308, 1.0), 20.0)


def test_clearance_grid_uses_129_square_points_for_default_3600m_box() -> None:
    """题设 3600m 方形在默认 20m 半径下应保持 129×129 个基础候选。"""

    assert len(clearance_grid((-1800.0, -1800.0, 1800.0, 1800.0))) == 129**2


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
