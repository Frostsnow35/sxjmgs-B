"""
Geometry utilities for the B-problem Q3/Q4 exploration.

All angles are in degrees unless otherwise stated.
A bearing measurement at point P with returned angle a and maximum error
e = 1 degree means that the true source lies in the cone

    angle(P -> X) in [a-e, a+e].

The cone is represented exactly by two half-planes.
"""
from __future__ import annotations

import math
import random
from typing import Iterable, Sequence

EPS = 1e-9
CLEAN_TOL = 1e-7
DEG = math.pi / 180.0


def norm_deg(a: float) -> float:
    """Normalize an angle into [0, 360)."""
    a = math.fmod(a, 360.0)
    if a < 0:
        a += 360.0
    return a


def clip_polygon(poly: list[tuple[float, float]], a: float, b: float, c: float,
                 eps: float = EPS) -> list[tuple[float, float]]:
    """Clip polygon by half-plane a*x + b*y >= c (Sutherland-Hodgman)."""
    if not poly:
        return []
    out: list[tuple[float, float]] = []
    n = len(poly)
    for i in range(n):
        s = poly[i]
        t = poly[(i + 1) % n]
        fs = a * s[0] + b * s[1] - c
        ft = a * t[0] + b * t[1] - c
        s_in = fs >= -eps
        t_in = ft >= -eps
        if s_in:
            out.append(s)
        if s_in != t_in:
            den = fs - ft
            if abs(den) > 1e-18:
                lam = fs / den
                out.append((s[0] + lam * (t[0] - s[0]),
                            s[1] + lam * (t[1] - s[1])))
    return out


def polygon_from_halfplanes(halfplanes: Sequence[tuple[float, float, float]],
                            big: float = 1e7) -> list[tuple[float, float]]:
    poly = [(-big, -big), (big, -big), (big, big), (-big, big)]
    for a, b, c in halfplanes:
        poly = clip_polygon(poly, a, b, c)
    return poly


def wedge_halfplanes(point: tuple[float, float], center_deg: float,
                     half_width_deg: float = 1.0) -> list[tuple[float, float, float]]:
    """Half-planes for the angular cone centered at `center_deg` with half width."""
    lo = math.radians(center_deg - half_width_deg)
    hi = math.radians(center_deg + half_width_deg)
    x, y = point
    return [
        # angle >= lo:  -sin(lo)*dx + cos(lo)*dy >= 0
        (-math.sin(lo), math.cos(lo), -math.sin(lo) * x + math.cos(lo) * y),
        # angle <= hi:   sin(hi)*dx - cos(hi)*dy >= 0
        (math.sin(hi), -math.cos(hi), math.sin(hi) * x - math.cos(hi) * y),
    ]


def disk_halfplanes(center: tuple[float, float], radius: float,
                    n: int = 180, expand: bool = True) -> list[tuple[float, float, float]]:
    """Half-planes of a regular n-gon containing the disk center/radius.

    The polygon is slightly larger than the disk (`expand=True`) so the disk is a
    subset of the polygon.  A feasible set built from such polygons is therefore a
    conservative superset of the true feasible set.
    """
    if expand:
        r = radius / math.cos(math.pi / n)
    else:
        r = radius
    x, y = center
    out = []
    for k in range(n):
        t = 2.0 * math.pi * k / n
        nx, ny = math.cos(t), math.sin(t)
        # inside disk: nx*(X-x)+ny*(Y-y) <= radius
        # => -nx*X - ny*Y >= -nx*x - ny*y - radius
        out.append((-nx, -ny, -nx * x - ny * y - r))
    return out


def clean_points(poly: Sequence[tuple[float, float]],
                 tol: float = CLEAN_TOL) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for p in poly:
        if all(math.dist(p, q) > tol for q in out):
            out.append(p)
    return out


def polygon_area(poly: Sequence[tuple[float, float]]) -> float:
    if len(poly) < 3:
        return 0.0
    s = 0.0
    for i in range(len(poly)):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % len(poly)]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


def point_in_convex_polygon(p: tuple[float, float],
                            poly: Sequence[tuple[float, float]],
                            eps: float = 1e-8) -> bool:
    """Assume CCW or CW consistently; test cross products have same sign."""
    if len(poly) < 3:
        return False
    sign = 0
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if abs(cr) <= eps:
            continue
        s = 1 if cr > 0 else -1
        if sign == 0:
            sign = s
        elif sign != s:
            return False
    return True


def point_to_segment_distance(p: tuple[float, float], a: tuple[float, float],
                              b: tuple[float, float]) -> float:
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.dist(p, a)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    return math.dist(p, (ax + t * dx, ay + t * dy))


def point_to_polygon_distance(p: tuple[float, float],
                              poly: Sequence[tuple[float, float]]) -> float:
    if point_in_convex_polygon(p, poly):
        return 0.0
    return min(point_to_segment_distance(p, poly[i], poly[(i + 1) % len(poly)])
               for i in range(len(poly)))


# ---------------------------------------------------------------------------
# Minimal enclosing circle (MEC)
# ---------------------------------------------------------------------------

def _mec_circle_two(a: tuple[float, float], b: tuple[float, float]):
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, math.dist(a, b) / 2.0)


def _mec_circle_three(a: tuple[float, float], b: tuple[float, float],
                      c: tuple[float, float]):
    ax, ay = a
    bx, by = b
    cx, cy = c
    d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-15:
        return None
    ux = ((ax * ax + ay * ay) * (by - cy) +
          (bx * bx + by * by) * (cy - ay) +
          (cx * cx + cy * cy) * (ay - by)) / d
    uy = ((ax * ax + ay * ay) * (cx - bx) +
          (bx * bx + by * by) * (ax - cx) +
          (cx * cx + cy * cy) * (bx - ax)) / d
    return (ux, uy, math.dist((ux, uy), a))


def min_enclosing_circle(points: Sequence[tuple[float, float]]):
    """Return (center, radius) of the smallest circle containing all points.

    Randomized Welzl algorithm.  For a convex polygon this is the same as the
    MEC of its vertex set.
    """
    pts = list(clean_points(points))
    if not pts:
        return (0.0, 0.0), 1e100
    random.Random(12345).shuffle(pts)

    def encloses(circle, p) -> bool:
        if circle is None:
            return False
        return math.dist((circle[0], circle[1]), p) <= circle[2] + 1e-8

    def rec(boundary: list, n: int):
        if n == 0 or len(boundary) == 3:
            if len(boundary) == 0:
                return (0.0, 0.0, 0.0)
            if len(boundary) == 1:
                return (boundary[0][0], boundary[0][1], 0.0)
            if len(boundary) == 2:
                return _mec_circle_two(boundary[0], boundary[1])
            c3 = _mec_circle_three(boundary[0], boundary[1], boundary[2])
            if c3 is None:
                # collinear: choose the two farthest boundary points
                pairs = [(0, 1), (0, 2), (1, 2)]
                i, j = max(pairs, key=lambda ij: math.dist(boundary[ij[0]], boundary[ij[1]]))
                return _mec_circle_two(boundary[i], boundary[j])
            return c3
        p = pts[n - 1]
        circle = rec(boundary, n - 1)
        if encloses(circle, p):
            return circle
        return rec(boundary + [p], n - 1)

    circle = rec([], len(pts))
    return (circle[0], circle[1]), circle[2]


def polygon_mec(poly: Sequence[tuple[float, float]]):
    return min_enclosing_circle(poly)


# ---------------------------------------------------------------------------
# Feasible polygon from bearing observations
# ---------------------------------------------------------------------------

def build_feasible_polygon(signals: Sequence[tuple[tuple[float, float], float]],
                           disk_centers: Sequence[tuple[float, float]] | None = None,
                           max_radius: float = 1500.0,
                           disk_n: int = 180) -> list[tuple[float, float]]:
    """Build a conservative convex feasible polygon for a source.

    `signals`: (measurement point, returned svd_deg) for every measurement that
    returned `direction`.  The true source is inside each cone and inside every
    disk of radius `max_radius` around a signal point (effective radius <= 1500).
    """
    if not signals:
        return []
    if disk_centers is None:
        disk_centers = [signals[0][0]]
    # Start from the first disk (a slightly expanded polygon containing the disk).
    halfplanes = disk_halfplanes(disk_centers[0], max_radius, n=disk_n)
    for center in disk_centers[1:]:
        halfplanes += disk_halfplanes(center, max_radius, n=max(60, disk_n // 2))
    for point, deg in signals:
        halfplanes += wedge_halfplanes(point, deg)
    return polygon_from_halfplanes(halfplanes)


# ---------------------------------------------------------------------------
# Covering a polygon by disks of radius `cover_radius` using a square grid.
# A square grid with spacing sqrt(2)*R has covering radius R.
# ---------------------------------------------------------------------------

def polygon_bbox(poly: Sequence[tuple[float, float]]):
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs), max(xs), min(ys), max(ys)


def cover_points_for_polygon(poly: Sequence[tuple[float, float]],
                             cover_radius: float = 20.0,
                             margin: float = 1.0) -> list[tuple[float, float]]:
    """Grid points whose `cover_radius` disks cover the whole polygon.

    Spacing = sqrt(2)*cover_radius guarantees that every point of the bounding
    box is within cover_radius of a grid point.  We keep a grid point when its
    disk intersects the polygon.
    """
    if not poly:
        return []
    xmin, xmax, ymin, ymax = polygon_bbox(poly)
    s = math.sqrt(2.0) * cover_radius
    points: list[tuple[float, float]] = []
    ixmin = math.floor((xmin - margin) / s)
    ixmax = math.ceil((xmax + margin) / s)
    iymin = math.floor((ymin - margin) / s)
    iymax = math.ceil((ymax + margin) / s)
    for iy in range(iymin, iymax + 1):
        for ix in range(ixmin, ixmax + 1):
            p = (ix * s, iy * s)
            if point_to_polygon_distance(p, poly) <= cover_radius + 1e-6:
                points.append(p)
    return points


def order_route(points: Sequence[tuple[float, float]],
                start: tuple[float, float]) -> list[tuple[float, float]]:
    """Greedy nearest-neighbour route + 2-opt improvement."""
    remaining = list(points)
    route: list[tuple[float, float]] = []
    cur = start
    while remaining:
        i = min(range(len(remaining)), key=lambda j: math.dist(cur, remaining[j]))
        cur = remaining.pop(i)
        route.append(cur)

    def total(r: list[tuple[float, float]]) -> float:
        s = 0.0
        c = start
        for p in r:
            s += math.dist(c, p)
            c = p
        return s

    best = route[:]
    best_len = total(best)
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 1):
            for j in range(i + 2, len(best)):
                new = best[:i] + best[i:j][::-1] + best[j:]
                ln = total(new)
                if ln < best_len - 1e-7:
                    best, best_len = new, ln
                    improved = True
    return best


def route_length(points: Sequence[tuple[float, float]],
                 start: tuple[float, float]) -> float:
    cur = start
    total = 0.0
    for p in points:
        total += math.dist(cur, p)
        cur = p
    return total
