"""问题三的纯几何保证策略，不执行网络或模拟器动作。"""

from __future__ import annotations

import math
from numbers import Real


Point = tuple[float, float]


def _finite_real(value: float, name: str) -> float:
    """校验并返回一个有限实数。"""

    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} 必须是有限实数")
    return float(value)


def _positive_finite(value: float, name: str) -> float:
    """校验并返回一个正的有限实数。"""

    numeric_value = _finite_real(value, name)
    if numeric_value <= 0.0:
        raise ValueError(f"{name} 必须为正的有限实数")
    return numeric_value


def _validated_point(point: Point, name: str) -> Point:
    """校验点恰有两个有限坐标。"""

    try:
        x, y = point
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} 必须包含两个坐标") from error
    return (_finite_real(x, f"{name}[0]"), _finite_real(y, f"{name}[1]"))


def initial_scan_points(radius_m: float = 1200.0) -> tuple[Point, ...]:
    """返回圆心及从 0° 起逆时针排列的正六边形首次扫描点。"""

    radius = _positive_finite(radius_m, "radius_m")
    vertices = tuple(
        (radius * math.cos(index * math.tau / 6.0), radius * math.sin(index * math.tau / 6.0))
        for index in range(6)
    )
    return ((0.0, 0.0), *vertices)


def worst_initial_scan_distance(
    target_radius_m: float = 1800.0, scan_radius_m: float = 1200.0
) -> float:
    """返回目标圆到首次七点扫描集最近距离的题设严格上界。"""

    target_radius = _positive_finite(target_radius_m, "target_radius_m")
    scan_radius = _positive_finite(scan_radius_m, "scan_radius_m")
    return math.sqrt(
        target_radius**2
        + scan_radius**2
        - 2.0 * target_radius * scan_radius * math.cos(math.pi / 6.0)
    )


def guaranteed_second_point(
    first: Point, bearing_deg: float, lateral_sign: float = 1.0
) -> Point:
    """按题设前向 750m、横向 ±600m 公式返回保证性二次测向点。"""

    first_x, first_y = _validated_point(first, "first")
    bearing = _finite_real(bearing_deg, "bearing_deg")
    sign = _finite_real(lateral_sign, "lateral_sign")
    if sign not in (-1.0, 1.0):
        raise ValueError("lateral_sign 只能为 -1.0 或 1.0")

    theta = math.radians(bearing)
    forward_x, forward_y = math.cos(theta), math.sin(theta)
    lateral_x, lateral_y = -forward_y, forward_x
    return (
        first_x + 750.0 * forward_x + 600.0 * sign * lateral_x,
        first_y + 750.0 * forward_y + 600.0 * sign * lateral_y,
    )


def _validated_bounds(bounds: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """校验并返回按 xmin、ymin、xmax、ymax 排列的有限边界。"""

    try:
        xmin, ymin, xmax, ymax = bounds
    except (TypeError, ValueError) as error:
        raise ValueError("bounds 必须包含 xmin、ymin、xmax、ymax") from error

    checked_bounds = (
        _finite_real(xmin, "xmin"),
        _finite_real(ymin, "ymin"),
        _finite_real(xmax, "xmax"),
        _finite_real(ymax, "ymax"),
    )
    if checked_bounds[0] > checked_bounds[2] or checked_bounds[1] > checked_bounds[3]:
        raise ValueError("bounds 必须满足 xmin <= xmax 且 ymin <= ymax")
    return checked_bounds


def _axis_grid(lower: float, upper: float, step: float) -> tuple[float, ...]:
    """以端点补齐方式生成相邻间隔不超过步长的一维候选轴。"""

    if lower == upper:
        return (lower,)

    full_steps = math.floor((upper - lower) / step)
    values = [lower + index * step for index in range(full_steps + 1)]
    if values[-1] != upper:
        values.append(upper)
    return tuple(values)


def clearance_grid(
    bounds: tuple[float, float, float, float], radius_m: float = 20.0
) -> tuple[Point, ...]:
    """生成覆盖边界的有限清除兜底候选网格，不声称其为高效策略。"""

    xmin, ymin, xmax, ymax = _validated_bounds(bounds)
    radius = _positive_finite(radius_m, "radius_m")
    step = radius * math.sqrt(2.0)
    x_values = _axis_grid(xmin, xmax, step)
    y_values = _axis_grid(ymin, ymax, step)
    return tuple((x, y) for x in x_values for y in y_values)
