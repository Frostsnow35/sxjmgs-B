"""Robust, offline planar geometry for CUMCM 2026 B questions 1 and 2.

The module deliberately has no simulator, filesystem, or network dependency.
Coordinates are metres, while input bearings are degrees measured counterclockwise
from east, as specified in the problem statement.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
import logging
import math
from typing import Final, Sequence


LOGGER = logging.getLogger(__name__)
Point = tuple[float, float]
Observation = tuple[float, float, float]
_TOLERANCE: Final[float] = 1e-8


@dataclass(frozen=True)
class _HalfPlane:
    """The closed half-plane ``normal · point >= offset``."""

    normal: Point
    offset: float


@dataclass(frozen=True)
class LocalizationResult:
    """Geometry summary of a closed intersection of bearing cones."""

    status: str
    vertices: tuple[Point, ...]
    diameter_m: float | None
    diameter_pairs: tuple[tuple[Point, Point], ...]
    diameter_circle_covers: bool | None
    cover_centers: tuple[Point, ...]


@dataclass(frozen=True)
class CandidateScore:
    """A reproducible question-2 score for one possible second detector point."""

    point: Point
    worst_case_sine: float
    max_distance_m: float
    guaranteed_receivable: bool


def locate_from_bearings(
    observations: Sequence[Observation], error_deg: float = 1.0
) -> LocalizationResult:
    """Intersect closed bearing cones and summarize the resulting convex set.

    Args:
        observations: Tuples ``(x_m, y_m, bearing_deg)`` for each detector.
        error_deg: Symmetric, deterministic angular error bound in degrees.

    Returns:
        A result labelled ``empty``, ``unbounded``, ``point``, ``segment``, or
        ``polygon``.  Diameter-related fields are ``None`` for empty/unbounded
        regions because no finite diameter is defined there.

    Raises:
        ValueError: If the observation list is empty or the error cone is invalid.
    """
    if not observations:
        raise ValueError("at least one bearing observation is required")
    if not 0.0 <= error_deg < 90.0:
        raise ValueError("error_deg must lie in [0, 90)")

    constraints = _bearing_halfplanes(observations, error_deg)
    candidates = _feasible_boundary_candidates(observations, constraints)
    if not candidates:
        LOGGER.info("bearing intersection is empty", extra={"observations": len(observations)})
        return LocalizationResult("empty", (), None, (), None, ())

    vertices = tuple(_convex_hull(candidates))
    if _has_nonzero_recession_direction(constraints):
        LOGGER.info("bearing intersection is unbounded", extra={"observations": len(observations)})
        return LocalizationResult("unbounded", vertices, None, (), None, ())

    if len(vertices) == 1:
        point = vertices[0]
        return LocalizationResult("point", vertices, 0.0, ((point, point),), True, (point,))
    if len(vertices) == 2:
        covers, diameter_m, centers = diameter_circle_covers(vertices)
        return LocalizationResult("segment", vertices, diameter_m, _diameter_pairs(vertices, diameter_m), covers, centers)

    covers, diameter_m, centers = diameter_circle_covers(vertices)
    return LocalizationResult(
        "polygon",
        vertices,
        diameter_m,
        _diameter_pairs(vertices, diameter_m),
        covers,
        centers,
    )


def diameter_circle_covers(vertices: Sequence[Point]) -> tuple[bool, float, tuple[Point, ...]]:
    """Test whether any circle based on a diameter pair covers all vertices.

    Convexity means that checking vertices is sufficient to check the full polygon.
    The returned centers correspond to every vertex pair attaining the diameter.
    """
    if not vertices:
        raise ValueError("at least one vertex is required")
    if len(vertices) == 1:
        return True, 0.0, (vertices[0],)

    diameter_m = max(math.dist(left, right) for left in vertices for right in vertices)
    pairs = _diameter_pairs(vertices, diameter_m)
    centers = tuple(((left[0] + right[0]) / 2.0, (left[1] + right[1]) / 2.0) for left, right in pairs)
    radius_m = diameter_m / 2.0
    covers = any(
        all(math.dist(center, vertex) <= radius_m + _TOLERANCE for vertex in vertices)
        for center in centers
    )
    return covers, diameter_m, centers


def intersection_sine(first_point: Point, second_point: Point, source_point: Point) -> float:
    """Return ``|sin(phi)|`` for the two bearings from detector points to a source.

    A value of one is orthogonal geometry; zero is collinear or undefined because
    the source coincides with a detector point.
    """
    first_vector = _subtract(source_point, first_point)
    second_vector = _subtract(source_point, second_point)
    first_length = _norm(first_vector)
    second_length = _norm(second_vector)
    if first_length <= _TOLERANCE or second_length <= _TOLERANCE:
        return 0.0
    score = abs(_cross(first_vector, second_vector)) / (first_length * second_length)
    return min(1.0, max(0.0, score))


def worst_case_intersection_sine(
    second_point: Point, first_point: Point, source_points: Sequence[Point]
) -> float:
    """Return the least favorable intersection-angle quality over source points."""
    if not source_points:
        raise ValueError("source_points must not be empty")
    return min(intersection_sine(first_point, second_point, source) for source in source_points)


def rank_second_points(
    first_point: Point,
    source_points: Sequence[Point],
    candidates: Sequence[Point],
    guaranteed_receive_radius_m: float = 1_000.0,
) -> tuple[CandidateScore, ...]:
    """Rank candidate second detectors by worst-case intersection angle.

    The primary key maximizes ``min |sin(phi)|``.  The secondary key minimizes the
    farthest candidate-source distance, making otherwise tied choices easier to
    receive.  ``guaranteed_receivable`` enforces the 1000 m lower reception bound
    from the statement without assuming unknown actual source ranges.
    """
    if not source_points:
        raise ValueError("source_points must not be empty")
    if guaranteed_receive_radius_m <= 0.0:
        raise ValueError("guaranteed_receive_radius_m must be positive")

    scored = []
    for candidate in candidates:
        max_distance_m = max(math.dist(candidate, source) for source in source_points)
        scored.append(
            CandidateScore(
                point=candidate,
                worst_case_sine=worst_case_intersection_sine(candidate, first_point, source_points),
                max_distance_m=max_distance_m,
                guaranteed_receivable=max_distance_m <= guaranteed_receive_radius_m + _TOLERANCE,
            )
        )
    return tuple(sorted(scored, key=lambda item: (-item.worst_case_sine, item.max_distance_m, item.point)))


def strict_second_point_kernel(
    first_point: Point,
    source_points: Sequence[Point],
    candidates: Sequence[Point],
    angle_tolerance_deg: float,
    guaranteed_receive_radius_m: float = 1_000.0,
) -> tuple[CandidateScore, ...]:
    """Return candidates in the sampled strict robust kernel ``K_gamma``.

    A retained point guarantees (over the supplied finite source proxy) both a
    second reception within the smallest possible reception radius and an
    intersection angle inside ``90 ± angle_tolerance_deg``.  An empty result is
    informative: callers should then report the soft ranking rather than claim a
    guarantee that the sampled uncertainty set cannot support.
    """
    if not 0.0 <= angle_tolerance_deg <= 90.0:
        raise ValueError("angle_tolerance_deg must lie in [0, 90]")
    sine_threshold = math.cos(math.radians(angle_tolerance_deg))
    ranked = rank_second_points(
        first_point=first_point,
        source_points=source_points,
        candidates=candidates,
        guaranteed_receive_radius_m=guaranteed_receive_radius_m,
    )
    return tuple(
        item
        for item in ranked
        if item.guaranteed_receivable and item.worst_case_sine >= sine_threshold - _TOLERANCE
    )


def sample_first_direction_region(
    first_point: Point,
    bearing_deg: float,
    grid_spacing_m: float,
    error_deg: float = 1.0,
    target_radius_m: float = 1_800.0,
    maximum_receive_radius_m: float = 1_500.0,
) -> tuple[Point, ...]:
    """Discretize the first-direction feasible region ``U_1`` for question 2.

    The returned points satisfy the task's target disk, the maximum possible
    reception radius for a successful first ``direction`` response, and the
    deterministic bearing-error cone.  This is a finite proxy for ``U_1``:
    conclusions based on it must state ``grid_spacing_m`` and should be checked
    under grid refinement before being treated as a decision recommendation.
    """
    if grid_spacing_m <= 0.0:
        raise ValueError("grid_spacing_m must be positive")
    if not 0.0 <= error_deg < 90.0:
        raise ValueError("error_deg must lie in [0, 90)")
    if target_radius_m <= 0.0 or maximum_receive_radius_m <= 0.0:
        raise ValueError("radii must be positive")

    sampled: list[Point] = []
    for point in sample_target_disk(grid_spacing_m, target_radius_m):
        distance_m = math.dist(point, first_point)
        if distance_m <= _TOLERANCE or distance_m > maximum_receive_radius_m + _TOLERANCE:
            continue
        actual_bearing_deg = math.degrees(math.atan2(point[1] - first_point[1], point[0] - first_point[0]))
        if _angular_distance_deg(actual_bearing_deg, bearing_deg) <= error_deg + _TOLERANCE:
            sampled.append(point)
    LOGGER.info("sampled first-direction region", extra={"sample_count": len(sampled), "grid_spacing_m": grid_spacing_m})
    return tuple(sampled)


def sample_target_disk(grid_spacing_m: float, target_radius_m: float = 1_800.0) -> tuple[Point, ...]:
    """Return an origin-centred square lattice clipped to the target disk."""
    if grid_spacing_m <= 0.0:
        raise ValueError("grid_spacing_m must be positive")
    if target_radius_m <= 0.0:
        raise ValueError("target_radius_m must be positive")
    grid_limit = math.floor(target_radius_m / grid_spacing_m)
    return tuple(
        (x_index * grid_spacing_m, y_index * grid_spacing_m)
        for x_index in range(-grid_limit, grid_limit + 1)
        for y_index in range(-grid_limit, grid_limit + 1)
        if math.hypot(x_index * grid_spacing_m, y_index * grid_spacing_m) <= target_radius_m + _TOLERANCE
    )


def _bearing_halfplanes(observations: Sequence[Observation], error_deg: float) -> tuple[_HalfPlane, ...]:
    """Convert each closed bearing cone into two angular and one forward half-plane."""
    constraints: list[_HalfPlane] = []
    for x_m, y_m, bearing_deg in observations:
        lower = _direction(bearing_deg - error_deg)
        upper = _direction(bearing_deg + error_deg)
        central = _direction(bearing_deg)
        source = (x_m, y_m)

        # cross(lower, point-source) >= 0
        lower_normal = (-lower[1], lower[0])
        # cross(point-source, upper) >= 0
        upper_normal = (upper[1], -upper[0])
        constraints.extend(
            (
                _HalfPlane(lower_normal, _dot(lower_normal, source)),
                _HalfPlane(upper_normal, _dot(upper_normal, source)),
                # Redundant for positive errors, but necessary when error_deg == 0.
                _HalfPlane(central, _dot(central, source)),
            )
        )
    return tuple(constraints)


def _feasible_boundary_candidates(
    observations: Sequence[Observation], constraints: Sequence[_HalfPlane]
) -> list[Point]:
    """Enumerate finite boundary intersections and retain only feasible ones."""
    candidates = [(x_m, y_m) for x_m, y_m, _ in observations]
    for left_index, left in enumerate(constraints):
        for right in constraints[left_index + 1 :]:
            point = _line_intersection(left, right)
            if point is not None:
                candidates.append(point)
    return _unique_points(point for point in candidates if _is_feasible(point, constraints))


def _line_intersection(left: _HalfPlane, right: _HalfPlane) -> Point | None:
    """Return the intersection of two boundary lines, if they are not parallel."""
    determinant = _cross(left.normal, right.normal)
    if abs(determinant) <= _TOLERANCE:
        return None
    x_m = (left.offset * right.normal[1] - left.normal[1] * right.offset) / determinant
    y_m = (left.normal[0] * right.offset - left.offset * right.normal[0]) / determinant
    return (x_m, y_m)


def _is_feasible(point: Point, constraints: Sequence[_HalfPlane]) -> bool:
    """Check all closed half-plane constraints with a small numerical tolerance."""
    return all(_dot(constraint.normal, point) >= constraint.offset - _TOLERANCE for constraint in constraints)


def _has_nonzero_recession_direction(constraints: Sequence[_HalfPlane]) -> bool:
    """Detect a nonzero direction satisfying every homogeneous half-plane.

    In two dimensions a nontrivial polyhedral recession cone has either an extreme
    ray perpendicular to a constraint normal or contains a constraint normal.  The
    finite candidate set below therefore suffices for this closed-cone test.
    """
    directions: list[Point] = [(1.0, 0.0), (0.0, 1.0)]
    for constraint in constraints:
        normal = constraint.normal
        directions.extend((normal, (-normal[0], -normal[1]), (-normal[1], normal[0]), (normal[1], -normal[0])))
    for direction in directions:
        length = _norm(direction)
        if length <= _TOLERANCE:
            continue
        unit_direction = (direction[0] / length, direction[1] / length)
        if all(_dot(constraint.normal, unit_direction) >= -_TOLERANCE for constraint in constraints):
            return True
    return False


def _diameter_pairs(vertices: Sequence[Point], diameter_m: float) -> tuple[tuple[Point, Point], ...]:
    """Return each unordered vertex pair whose distance equals the diameter."""
    if len(vertices) == 1:
        return ((vertices[0], vertices[0]),)
    pairs: list[tuple[Point, Point]] = []
    for left_index, left in enumerate(vertices):
        for right in vertices[left_index + 1 :]:
            if abs(math.dist(left, right) - diameter_m) <= _TOLERANCE:
                pairs.append((left, right))
    return tuple(pairs)


def _convex_hull(points: Sequence[Point]) -> list[Point]:
    """Return a counterclockwise monotone-chain hull without duplicate endpoints."""
    unique = sorted(_unique_points(points))
    if len(unique) <= 1:
        return unique

    def build_half(sequence: Sequence[Point]) -> list[Point]:
        half: list[Point] = []
        for point in sequence:
            while len(half) >= 2 and _cross(_subtract(half[-1], half[-2]), _subtract(point, half[-1])) <= _TOLERANCE:
                half.pop()
            half.append(point)
        return half

    lower = build_half(unique)
    upper = build_half(list(reversed(unique)))
    return lower[:-1] + upper[:-1]


def _unique_points(points: Iterable[Point]) -> list[Point]:
    """Deduplicate a finite iterable of points using the module tolerance."""
    unique: list[Point] = []
    for point in points:
        if not any(math.dist(point, existing) <= _TOLERANCE for existing in unique):
            unique.append(point)
    return unique


def _direction(angle_deg: float) -> Point:
    """Convert a bearing in degrees to its east/north unit direction vector."""
    angle_rad = math.radians(angle_deg % 360.0)
    return (math.cos(angle_rad), math.sin(angle_rad))


def _angular_distance_deg(left_deg: float, right_deg: float) -> float:
    """Return the smallest unsigned difference between two degree bearings."""
    return abs((left_deg - right_deg + 180.0) % 360.0 - 180.0)


def _dot(left: Point, right: Point) -> float:
    """Return the two-dimensional dot product."""
    return left[0] * right[0] + left[1] * right[1]


def _cross(left: Point, right: Point) -> float:
    """Return the scalar two-dimensional cross product."""
    return left[0] * right[1] - left[1] * right[0]


def _subtract(left: Point, right: Point) -> Point:
    """Return ``left - right``."""
    return (left[0] - right[0], left[1] - right[1])


def _norm(vector: Point) -> float:
    """Return the Euclidean norm of a two-dimensional vector."""
    return math.hypot(vector[0], vector[1])
