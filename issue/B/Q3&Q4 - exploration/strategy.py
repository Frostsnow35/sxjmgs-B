"""
Automatic search / localization / clearing strategies for Q3 and Q4.

The strategies work against any object with the simulator action interface:

    iface.measure(pos, channel) -> object with .measure_result / .svd_deg
    iface.clear(pos, channel)   -> object with .clear_result
    iface.pos                   -> current position
    iface.current_channel       -> current receiver channel
    iface.virtual_time          -> current virtual time

They are deterministic (apart from geometry randomisation in Welzl, which
only affects tie-breaking and not the result).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import geometry as geo
import joint_feasibility as joint

CHANNELS = list(range(1, 21))
CLEAR_RADIUS = 20.0
MAX_Q3_ITER = 15
MAX_Q4_ITER = 6
PERP_PROBE_DIST = 100.0
SECTOR_TRACK_HALF_ANGLE = 0.5
SECTOR_TRACK_STEP = 30.0

# ---------------------------------------------------------------------------
# Survey point sets
# ---------------------------------------------------------------------------

def q3_survey_points() -> list[tuple[float, float]]:
    """7 points covering the radius-1800 disk with covering radius 900.

    One point at the origin and six points on a regular hexagon of radius
    900*sqrt(3).  For an omni source r>=1000, the nearest survey point is at
    most 900 m away, so every source is detected.
    """
    a = 900.0 * math.sqrt(3.0)
    pts = [(0.0, 0.0)]
    for k in range(6):
        t = math.pi / 3.0 * k
        pts.append((a * math.cos(t), a * math.sin(t)))
    return pts


def _closest_point_on_triangle(p, a, b, c):
    """Closest point on triangle abc to p (Ericson's barycentric method)."""
    ab = (b[0] - a[0], b[1] - a[1])
    ac = (c[0] - a[0], c[1] - a[1])
    ap = (p[0] - a[0], p[1] - a[1])
    d1 = ab[0] * ap[0] + ab[1] * ap[1]
    d2 = ac[0] * ap[0] + ac[1] * ap[1]
    if d1 <= 0.0 and d2 <= 0.0:
        return a

    bp = (p[0] - b[0], p[1] - b[1])
    d3 = ab[0] * bp[0] + ab[1] * bp[1]
    d4 = ac[0] * bp[0] + ac[1] * bp[1]
    if d3 >= 0.0 and d4 <= d3:
        return b

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        return (a[0] + v * ab[0], a[1] + v * ab[1])

    cp = (p[0] - c[0], p[1] - c[1])
    d5 = ab[0] * cp[0] + ab[1] * cp[1]
    d6 = ac[0] * cp[0] + ac[1] * cp[1]
    if d6 >= 0.0 and d5 <= d6:
        return c

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        return (a[0] + w * ac[0], a[1] + w * ac[1])

    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return (b[0] + w * (c[0] - b[0]), b[1] + w * (c[1] - b[1]))
    return a  # numerical fallback; not reached for non-degenerate triangles


def q4_survey_points(arena_r: float = 1800.0) -> list[tuple[float, float]]:
    """Generate the 25-point two-ring survey mesh for the mixed case.

    The mesh is a regular triangulation certificate:
      * outer ring: 12 points on radius `arena_r / cos(pi/12)`, phase 15°,
        so the 12-gon hull has in-radius exactly `arena_r`;
      * inner ring: 12 points on radius 950 m, phase 0°;
      * one point at the origin.

    The explicit triangulation (12 centre fan triangles + 24 annulus
    triangles) has longest edge 977.303 m < 1000 m and covers the whole
    arena disk.  Therefore every source lies in a triangle whose three
    vertices are within 1000 m of it; any directional half-plane contains at
    least one vertex of that triangle, which gives the discovery guarantee.
    """
    outer_r = arena_r / math.cos(math.pi / 12)
    inner_r = 950.0
    pts = [(0.0, 0.0)]
    for k in range(12):
        t = math.radians(30.0 * k)
        pts.append((inner_r * math.cos(t), inner_r * math.sin(t)))
    for k in range(12):
        t = math.radians(15.0 + 30.0 * k)
        pts.append((outer_r * math.cos(t), outer_r * math.sin(t)))
    assert len(pts) == 25
    return pts


# ---------------------------------------------------------------------------
# Survey / scanning
# ---------------------------------------------------------------------------

@dataclass
class ChannelData:
    signals: list = field(default_factory=list)   # list of (pos, svd_deg)
    near_points: list = field(default_factory=list)
    no_signal_points: list = field(default_factory=list)
    first_signal_pos: tuple | None = None


def _scan_at(iface, point: tuple[float, float], data: dict[int, ChannelData],
             enough: set[int], min_signals: int):
    """Measure channels at one point, starting from the current channel.

    `enough` contains channels for which `min_signals` useful bearings have
    already been collected; they are skipped at later survey points because
    discovery is already guaranteed and further survey scans are optional.
    """
    cur = iface.current_channel
    needed = [c for c in CHANNELS if c not in enough]
    if not needed:
        return
    # On the 1..20 channel line, the shortest switch order from `cur` is to
    # finish the nearer side first and then sweep the other side.
    lo, hi = min(needed), max(needed)
    if cur <= lo:
        order = sorted(needed)
    elif cur >= hi:
        order = sorted(needed, reverse=True)
    else:
        left_first = (cur - lo) + (hi - lo)
        right_first = (hi - cur) + (hi - lo)
        if left_first <= right_first:
            order = sorted((c for c in needed if c < cur), reverse=True) + \
                sorted(c for c in needed if c >= cur)
        else:
            order = sorted(c for c in needed if c > cur) + \
                sorted((c for c in needed if c <= cur), reverse=True)
    for ch in order:
        res = iface.measure(point, ch)
        d = data[ch]
        if res.measure_result == "direction":
            d.signals.append((point, res.svd_deg))
            if d.first_signal_pos is None:
                d.first_signal_pos = point
            if len(d.signals) >= min_signals:
                enough.add(ch)
        elif res.measure_result == "near":
            d.near_points.append(point)
            if d.first_signal_pos is None:
                d.first_signal_pos = point
            enough.add(ch)
        else:
            d.no_signal_points.append(point)


def survey(iface, points: list[tuple[float, float]],
           start: tuple[float, float] | None = None,
           min_signals: int = 20):
    """Visit `points`, scanning channels at each point.

    `min_signals=20` means scan every channel at every point (full survey).
    Smaller values stop scanning a channel after `min_signals` useful bearings
    have been collected; this saves detection/switch time without weakening
    the discovery guarantee.
    """
    data = {ch: ChannelData() for ch in CHANNELS}
    enough: set[int] = set()
    if start is None:
        start = iface.pos
    route = geo.order_route(points, start)
    for p in route:
        _scan_at(iface, p, data, enough, min_signals)
    return data


# ---------------------------------------------------------------------------
# Localization and clearing
# ---------------------------------------------------------------------------

def _rebuild_poly(signals: list, disk_centers: list | None = None):
    if not signals:
        return []
    if disk_centers is None:
        disk_centers = [p for p, _ in signals]
    return geo.build_feasible_polygon(signals, disk_centers)


def _clear_at(iface, point: tuple[float, float], channel: int) -> bool:
    res = iface.clear(point, channel)
    return getattr(res, "clear_result", None) == "success"


def _safe_negative_points(signals: list, no_signal_points: list) -> list[tuple[float, float]]:
    """Return the convex-hull safe negatives from the joint model module."""

    return joint.safe_negative_points(signals, no_signal_points)


def _joint_projection_summary(poly, signals, no_signal_points):
    """Return sampled joint-projection center, radius and count, if nonempty.

    The summary is an action-ranking hint only.  Its radius does not bound the
    full feasible set, so callers must not use it to remove the conservative
    polygon or to suppress coverage fallback.
    """

    samples = joint.project_positions(poly, signals, no_signal_points, max_cell_m=25.0)
    if not samples:
        return None
    center, radius = geo.min_enclosing_circle([sample.position for sample in samples])
    return center, radius, len(samples)


def _sector_cover_points(point: tuple[float, float], center_deg: float,
                         length: float = 1505.0) -> list[tuple[float, float]]:
    """Cover a 2-degree cone with two radial tracks.

    Tracks at `center_deg +- 0.5` deg and clear points every 30 m keep the
    worst-case distance to any point of the cone below 20 m:
    lateral error <= 1500*sin(0.5 deg) < 13.1 m, along-track gap <= 15 m.
    """
    pts = []
    for offset in (SECTOR_TRACK_HALF_ANGLE, -SECTOR_TRACK_HALF_ANGLE):
        a = math.radians(center_deg + offset)
        u = (math.cos(a), math.sin(a))
        s = 0.0
        while s <= length + 1e-9:
            pts.append((point[0] + s * u[0], point[1] + s * u[1]))
            s += SECTOR_TRACK_STEP
    return pts


def _cover_and_clear(iface, poly, channel: int, signals=None,
                     safe_negatives=None, priority_center=None):
    """Guaranteed fallback: cover the conservative feasible set by clear disks."""
    if not poly and signals:
        # Numeric clipping should never produce an empty polygon when signals
        # exist, but reconstruct the one-cone fallback if it does.
        poly = _rebuild_poly([signals[0]], [signals[0][0]])
    if not poly:
        return False
    if signals and len(signals) == 1:
        pts = _sector_cover_points(signals[0][0], signals[0][1])
    else:
        pts = geo.cover_points_for_polygon(poly, CLEAR_RADIUS)
    # Safe negative information: for Q inside the signal convex hull, no_signal
    # means |G-Q| > 1000.  A clear candidate whose whole 20 m disk lies inside
    # B(Q,1000) cannot be needed to cover the true source.
    if safe_negatives:
        pts = [p for p in pts
               if all(math.dist(p, q) > 980.0 + 1e-7
                      for q in safe_negatives)]
    if not pts:
        # Degenerate polygon: clear at its first vertex.
        pts = [poly[0]]
    if priority_center is None:
        route = geo.order_route(pts, iface.pos)
    else:
        # The joint projection is only an ordering prior.  All candidates stay
        # in the route, preserving the radius-20 covering certificate.
        first_index = min(
            range(len(pts)), key=lambda index: math.dist(pts[index], priority_center)
        )
        first = pts.pop(first_index)
        route = [first, *geo.order_route(pts, first)]
    for p in route:
        if _clear_at(iface, p, channel):
            return True
    return False


def clear_channel_q3(iface, channel: int, d: ChannelData) -> bool:
    """Set-membership localization for an omni source.

    F = intersection of all bearing cones and the radius-1500 disk constraint.
    At every iteration the MEC center c of F has d(c,G) <= R(F) <= 750 m
    (< 1000 m <= effective radius), so a measurement at c is guaranteed to be
    signal/near for an omni source.  When R(F) <= 20 m, clear at c.
    """
    # Already very close to a survey point.
    if d.near_points:
        p = min(d.near_points, key=lambda q: math.dist(iface.pos, q))
        return _clear_at(iface, p, channel)

    signals = list(d.signals)
    disk_centers = [p for p, _ in signals]
    poly = _rebuild_poly(signals, disk_centers)
    if not poly:
        return False

    for _ in range(MAX_Q3_ITER):
        (cx, cy), radius = geo.min_enclosing_circle(poly)
        if radius <= CLEAR_RADIUS + 1e-6:
            if _clear_at(iface, (cx, cy), channel):
                return True
            return _cover_and_clear(iface, poly, channel, signals)

        res = iface.measure((cx, cy), channel)
        if res.measure_result == "near":
            if _clear_at(iface, (cx, cy), channel):
                return True
            return _cover_and_clear(iface, poly, channel, signals)
        if res.measure_result == "direction":
            signals.append(((cx, cy), res.svd_deg))
            disk_centers.append((cx, cy))
            poly = _rebuild_poly(signals, disk_centers)
            if not poly:
                poly = _rebuild_poly(signals[:1], [signals[0][0]])
            continue
        # For an omni source this should not happen.  Fall back to guaranteed
        # grid covering of the current conservative feasible set.
        return _cover_and_clear(iface, poly, channel, signals)

    return _cover_and_clear(iface, poly, channel, signals)


def clear_channel_q4(iface, channel: int, d: ChannelData) -> bool:
    """Strategy for Q4 (omni + directional unknown).

    Observations that returned a bearing are valid for both source types, so
    the intersection polygon F is still a conservative feasible set.  If F is
    already small, clear at its MEC center.  Otherwise we try a few bearing
    measurements at the MEC center; if they return a signal they shrink F.  If
    a measurement returns no_signal (the point is outside the directional
    sector), we stop measuring and cover the *valid* polygon F with clear
    disks.  /clear works independently of the source orientation.
    """
    if d.near_points:
        p = min(d.near_points, key=lambda q: math.dist(iface.pos, q))
        return _clear_at(iface, p, channel)

    signals = list(d.signals)
    disk_centers = [p for p, _ in signals]
    no_signal_points = list(d.no_signal_points)

    # One-signal directional sources: acquire a second bearing with two
    # perpendicular probes.  At least one of the two opposite perpendicular
    # directions has non-negative dot product with the unknown sector normal,
    # so it stays inside the sector; the other may be no_signal and is ignored.
    # This is only an acceleration attempt: if both probes fail we still keep
    # the original conservative polygon and fall back to sector covering.
    if len(signals) == 1:
        (px, py), alpha = signals[0]
        n = (math.cos(math.radians(alpha + 90.0)),
             math.sin(math.radians(alpha + 90.0)))
        for sgn in (1.0, -1.0):
            q = (px + sgn * PERP_PROBE_DIST * n[0],
                 py + sgn * PERP_PROBE_DIST * n[1])
            res = iface.measure(q, channel)
            if res.measure_result == "near":
                if _clear_at(iface, q, channel):
                    return True
            elif res.measure_result == "direction":
                signals.append((q, res.svd_deg))
                disk_centers.append(q)
            else:
                no_signal_points.append(q)

    safe_negatives = _safe_negative_points(signals, no_signal_points)

    poly = _rebuild_poly(signals, disk_centers)
    if not poly:
        return False

    projection = _joint_projection_summary(poly, signals, no_signal_points)

    for _ in range(MAX_Q4_ITER):
        (cx, cy), radius = geo.min_enclosing_circle(poly)
        action_point = (cx, cy)
        if radius <= CLEAR_RADIUS + 1e-6:
            if _clear_at(iface, (cx, cy), channel):
                return True
            priority_center = projection[0] if projection is not None else None
            return _cover_and_clear(
                iface, poly, channel, signals, safe_negatives, priority_center
            )

        # Cheap speculative clear when the feasible disk is only slightly too
        # large.  We are already at c; a failed clear costs 3 s and does not
        # weaken the later guaranteed fallback.
        if radius <= 25.0:
            if _clear_at(iface, (cx, cy), channel):
                return True

        res = iface.measure(action_point, channel)
        if res.measure_result == "near":
            if _clear_at(iface, action_point, channel):
                return True
            priority_center = projection[0] if projection is not None else None
            return _cover_and_clear(
                iface, poly, channel, signals, safe_negatives, priority_center
            )
        if res.measure_result == "direction":
            signals.append((action_point, res.svd_deg))
            disk_centers.append(action_point)
            safe_negatives = _safe_negative_points(signals, no_signal_points)
            poly = _rebuild_poly(signals, disk_centers)
            if not poly:
                poly = _rebuild_poly(signals[:1], [signals[0][0]])
            projection = _joint_projection_summary(poly, signals, no_signal_points)
            continue
        # no_signal: c is outside the sector (or outside range).  c may still
        # be within 20 m of the source; one cheap clear attempt often closes
        # borderline cases before the guaranteed grid fallback.
        no_signal_points.append(action_point)
        safe_negatives = _safe_negative_points(signals, no_signal_points)
        projection = _joint_projection_summary(poly, signals, no_signal_points)
        if _clear_at(iface, action_point, channel):
            return True
        break

    priority_center = projection[0] if projection is not None else None
    return _cover_and_clear(iface, poly, channel, signals, safe_negatives, priority_center)


def _first_action_point(d: ChannelData, mode: str = "q3") -> tuple[float, float]:
    if d.near_points:
        return min(d.near_points, key=lambda q: (q[0], q[1]))
    if d.signals:
        if mode == "q4" and len(d.signals) == 1:
            # The actual next actions for a one-signal Q4 channel are the two
            # perpendicular probes around this point.
            return d.signals[0][0]
        poly = _rebuild_poly(d.signals, [p for p, _ in d.signals])
        if poly:
            return geo.min_enclosing_circle(poly)[0]
        return d.signals[0][0]
    return (0.0, 0.0)


def clear_all(iface, mode: str, data: dict[int, ChannelData]) -> tuple[int, int]:
    """Clear all detected channels.

    The first action point of every channel is known before any additional
    measurement.  With at most 16 sources, an exact Held-Karp shortest path
    through these first action points is computed, which is better than a
    nearest-neighbour order and still inexpensive.
    """
    active = [ch for ch in CHANNELS
              if data[ch].signals or data[ch].near_points]
    entry = {ch: _first_action_point(data[ch], mode) for ch in active}
    ordered_points = geo.order_points_exact_tsp(
        [entry[ch] for ch in active], iface.pos)
    # Map points back to channels in the optimal order.
    remaining = set(active)
    ordered_channels = []
    for p in ordered_points:
        best = min(remaining, key=lambda ch: math.dist(entry[ch], p))
        ordered_channels.append(best)
        remaining.remove(best)

    cleared = 0
    failed = 0
    for ch in ordered_channels:
        ok = (clear_channel_q3(iface, ch, data[ch]) if mode == "q3"
              else clear_channel_q4(iface, ch, data[ch]))
        if ok:
            cleared += 1
        else:
            failed += 1
    return cleared, failed


def run_q3(iface) -> dict:
    pts = q3_survey_points()
    data = survey(iface, pts, iface.pos, min_signals=2)
    cleared, failed = clear_all(iface, "q3", data)
    return {
        "survey_points": pts,
        "data": data,
        "cleared": cleared,
        "failed": failed,
        "active_channels": [ch for ch in CHANNELS if data[ch].signals or data[ch].near_points],
    }


def run_q4(iface) -> dict:
    pts = q4_survey_points()
    data = survey(iface, pts, iface.pos, min_signals=2)
    cleared, failed = clear_all(iface, "q4", data)
    return {
        "survey_points": pts,
        "data": data,
        "cleared": cleared,
        "failed": failed,
        "active_channels": [ch for ch in CHANNELS if data[ch].signals or data[ch].near_points],
    }
