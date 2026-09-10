"""问题三的纯几何保证策略，不执行网络或模拟器动作。

清除兜底网格最多生成 ``MAX_CLEARANCE_CANDIDATES`` 个候选，以免病态边界
耗尽资源；题设最大约 3600m 的方形清除框在默认端点均分网格和 20m 半径下为
129×129，即 16641 个候选，低于此上限。
"""

from __future__ import annotations

import math
from numbers import Real


Point = tuple[float, float]

# 单次清除兜底网格的明确资源上限，非高效策略的最优性声明。
MAX_CLEARANCE_CANDIDATES = 100_000


def _finite_real(value: float, name: str) -> float:
    """校验并返回一个有限实数。"""

    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(f"{name} 必须是有限实数")
    try:
        numeric_value = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"{name} 必须是有限实数") from error
    if not math.isfinite(numeric_value):
        raise ValueError(f"{name} 必须是有限实数")
    return numeric_value


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
    return math.hypot(
        target_radius - scan_radius * math.cos(math.pi / 6.0),
        scan_radius * math.sin(math.pi / 6.0),
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


def _ensure_candidate_limit(candidate_count: int) -> None:
    """在构造候选前拒绝超过明确资源上限的网格。"""

    if candidate_count > MAX_CLEARANCE_CANDIDATES:
        raise ValueError(
            f"清除兜底候选数量超过上限 {MAX_CLEARANCE_CANDIDATES}，拒绝生成"
        )


def _axis_grid(lower: float, upper: float, step: float) -> tuple[float, ...]:
    """以分段栈补齐实际浮点间隔，顺序生成可证明覆盖的一维候选轴。"""

    if lower == upper:
        return (lower,)

    span = upper - lower
    if not math.isfinite(span):
        _ensure_candidate_limit(MAX_CLEARANCE_CANDIDATES + 1)
    maximum_span = step * (MAX_CLEARANCE_CANDIDATES - 1)
    if math.isfinite(maximum_span) and span > maximum_span:
        _ensure_candidate_limit(MAX_CLEARANCE_CANDIDATES + 1)

    interval_count = max(1, math.ceil(span / step))
    _ensure_candidate_limit(interval_count + 1)
    values = [lower]
    for index in range(1, interval_count):
        candidate = lower + span * (index / interval_count)
        if candidate <= values[-1]:
            continue
        if candidate >= upper:
            break
        values.append(candidate)
    if values[-1] != upper:
        values.append(upper)

    pending_segments = [
        (values[index], values[index + 1])
        for index in range(len(values) - 2, -1, -1)
    ]
    refined_values = [values[0]]
    segment_count = len(pending_segments)
    while pending_segments:
        left, right = pending_segments.pop()
        if right - left <= step:
            refined_values.append(right)
            continue
        midpoint = left + (right - left) / 2.0
        if not left < midpoint < right:
            raise ValueError("浮点分辨率不足，无法生成满足覆盖保证的清除候选")
        segment_count += 1
        _ensure_candidate_limit(segment_count + 1)
        pending_segments.append((midpoint, right))
        pending_segments.append((left, midpoint))
    return tuple(refined_values)


def clearance_grid(
    bounds: tuple[float, float, float, float], radius_m: float = 20.0
) -> tuple[Point, ...]:
    """生成有数量上限且按实际浮点间隔覆盖边界的清除兜底候选网格。"""

    xmin, ymin, xmax, ymax = _validated_bounds(bounds)
    radius = _positive_finite(radius_m, "radius_m")
    step = radius * math.sqrt(2.0)
    x_values = _axis_grid(xmin, xmax, step)
    y_values = _axis_grid(ymin, ymax, step)
    _ensure_candidate_limit(len(x_values) * len(y_values))
    return tuple((x, y) for x in x_values for y in y_values)
