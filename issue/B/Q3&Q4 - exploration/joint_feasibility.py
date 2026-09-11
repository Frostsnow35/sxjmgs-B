"""Semi-analytic location--orientation feasibility tools for Q4.

The module is deliberately independent of the simulator.  Bearing cones and
the radius-1500 upper bounds are handled by the caller's conservative polygon;
this module asks the complementary question: given a candidate source position,
can one directional half-plane and one effective radius explain all observed
signal/no-signal outcomes?
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

import geometry as geo


Point = tuple[float, float]
Signal = tuple[Point, float]
ANGLE_EPS = 1e-9
RADIUS_MIN = 1000.0
RADIUS_MAX = 1500.0


@dataclass(frozen=True)
class FeasibilityResult:
    """Existential feasibility result for one candidate source location."""

    feasible: bool
    r_min: float
    witness_angle_deg: float | None
    arc_count: int


@dataclass(frozen=True)
class ProjectedPoint:
    """A retained position-grid point and its analytic orientation witness."""

    position: Point
    feasibility: FeasibilityResult


def _norm_angle(angle_deg: float) -> float:
    """Map an angle to [0, 360)."""

    return angle_deg % 360.0


def _direction_angle(vector: Point) -> float:
    """Return the polar angle of a nonzero vector in degrees."""

    return _norm_angle(math.degrees(math.atan2(vector[1], vector[0])))


def _midpoint_ccw(left: float, right: float) -> float:
    """Return midpoint of the counter-clockwise circular interval left -> right."""

    return _norm_angle(left + ((_norm_angle(right - left)) / 2.0))


def _orientation_candidates(vectors: Sequence[Point]) -> list[float]:
    """Return interval boundaries and open-arc representatives on S^1."""

    boundaries = {0.0}
    for vx, vy in vectors:
        if math.hypot(vx, vy) <= ANGLE_EPS:
            continue
        centre = _direction_angle((vx, vy))
        boundaries.add(_norm_angle(centre - 90.0))
        boundaries.add(_norm_angle(centre + 90.0))
    ordered = sorted(boundaries)
    candidates = list(ordered)
    for index, left in enumerate(ordered):
        candidates.append(_midpoint_ccw(left, ordered[(index + 1) % len(ordered)]))
    return candidates


def _feasible_arc_count(vectors: Sequence[Point], valid) -> int:
    """Count connected feasible arcs from the semicircle-boundary arrangement."""

    boundaries = {0.0}
    for vx, vy in vectors:
        if math.hypot(vx, vy) <= ANGLE_EPS:
            continue
        centre = _direction_angle((vx, vy))
        boundaries.add(_norm_angle(centre - 90.0))
        boundaries.add(_norm_angle(centre + 90.0))
    ordered = sorted(boundaries)
    if len(ordered) == 1:
        return int(valid(ordered[0]))

    open_valid = [
        valid(_midpoint_ccw(left, ordered[(index + 1) % len(ordered)]))
        for index, left in enumerate(ordered)
    ]
    if all(open_valid):
        return 1
    count = sum(
        current and not open_valid[(index - 1) % len(open_valid)]
        for index, current in enumerate(open_valid)
    )
    # A closed positive-half-plane boundary can be feasible as an isolated
    # point when adjacent open arcs are both infeasible.
    for index, boundary in enumerate(ordered):
        if valid(boundary) and not open_valid[index] and not open_valid[index - 1]:
            count += 1
    return count


def safe_negative_points(
    signals: Sequence[Signal], no_signal_points: Sequence[Point]
) -> list[Point]:
    """Return negatives inside the convex hull of positive directional signals.

    Signal points lie in the unknown directional source half-plane.  Convexity
    puts every point of their convex hull in the same half-plane.  Therefore a
    no-signal at such a point cannot be explained by direction and necessarily
    has source distance strictly greater than the source's radius, hence 1000 m.
    """

    if len(signals) < 3 or not no_signal_points:
        return []
    hull = geo.convex_hull([point for point, _ in signals])
    if len(hull) < 3:
        return []
    return [
        point for point in no_signal_points
        if geo.point_in_convex_polygon(point, hull, eps=1e-7)
    ]


def feasible_orientation(
    position: Point, signals: Sequence[Signal], no_signal_points: Sequence[Point]
) -> FeasibilityResult:
    """Test whether one radius and one directional half-plane explain outcomes.

    For a fixed source position, choosing the smallest radius compatible with
    all positive signals is most permissive for no-signal observations.  Thus
    the continuous radius variable reduces to ``r_min``.  Orientation remains
    continuous but is checked exactly through the finite arrangement of its
    semicircle boundaries; no orientation grid is used.
    """

    if not signals:
        return FeasibilityResult(False, math.inf, None, 0)

    px, py = position
    positive_vectors = [(sx - px, sy - py) for (sx, sy), _ in signals]
    r_min = max(RADIUS_MIN, *(math.hypot(vx, vy) for vx, vy in positive_vectors))
    if r_min > RADIUS_MAX + ANGLE_EPS:
        return FeasibilityResult(False, r_min, None, 0)

    # Convex-hull negatives give the range-only corollary before the generic
    # distance-or-orientation treatment below.
    safe_negatives = safe_negative_points(signals, no_signal_points)
    if any(math.dist(position, q) <= RADIUS_MIN + ANGLE_EPS for q in safe_negatives):
        return FeasibilityResult(False, r_min, None, 0)

    negative_vectors = [
        (qx - px, qy - py)
        for qx, qy in no_signal_points
        if math.hypot(qx - px, qy - py) <= r_min + ANGLE_EPS
    ]
    all_vectors = [*positive_vectors, *negative_vectors]

    def valid(angle_deg: float) -> bool:
        nx = math.cos(math.radians(angle_deg))
        ny = math.sin(math.radians(angle_deg))
        # A received signal is in the closed directional half-plane.
        if any(nx * vx + ny * vy < -ANGLE_EPS for vx, vy in positive_vectors):
            return False
        # An in-range no-signal must lie in the strictly opposite half-plane.
        return all(nx * vx + ny * vy < -ANGLE_EPS for vx, vy in negative_vectors)

    candidates = _orientation_candidates(all_vectors)
    valid_candidates = [angle for angle in candidates if valid(angle)]
    if not valid_candidates:
        return FeasibilityResult(False, r_min, None, 0)

    # Every connected open feasible arc contributes a valid midpoint.  Isolated
    # closed-boundary witnesses are counted as one component when applicable.
    return FeasibilityResult(
        True,
        r_min,
        valid_candidates[0],
        _feasible_arc_count(all_vectors, valid),
    )


def project_positions(
    poly: Sequence[Point],
    signals: Sequence[Signal],
    no_signal_points: Sequence[Point],
    max_cell_m: float = 25.0,
) -> list[ProjectedPoint]:
    """Project joint feasibility onto a deterministic location grid in ``poly``.

    This function intentionally returns a finite *sample*, rather than an inner
    or outer geometric approximation.  Callers may use it to rank actions but
    must retain the conservative polygon cover as their correctness fallback.
    """

    if len(poly) < 3:
        return []
    if not math.isfinite(max_cell_m) or max_cell_m <= 0.0:
        raise ValueError("max_cell_m must be a positive finite distance")

    xmin, xmax, ymin, ymax = geo.polygon_bbox(poly)
    ix_min = math.ceil(xmin / max_cell_m - ANGLE_EPS)
    ix_max = math.floor(xmax / max_cell_m + ANGLE_EPS)
    iy_min = math.ceil(ymin / max_cell_m - ANGLE_EPS)
    iy_max = math.floor(ymax / max_cell_m + ANGLE_EPS)
    projected: list[ProjectedPoint] = []
    for ix in range(ix_min, ix_max + 1):
        x = ix * max_cell_m
        for iy in range(iy_min, iy_max + 1):
            position = (x, iy * max_cell_m)
            if not geo.point_in_convex_polygon(position, poly, eps=1e-7):
                continue
            result = feasible_orientation(position, signals, no_signal_points)
            if result.feasible:
                projected.append(ProjectedPoint(position, result))
    return projected
