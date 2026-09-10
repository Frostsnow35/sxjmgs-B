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

CHANNELS = list(range(1, 21))
CLEAR_RADIUS = 20.0
MAX_Q3_ITER = 15
MAX_Q4_ITER = 6

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


def q4_survey_points() -> list[tuple[float, float]]:
    """31-point triangular lattice survey for the mixed omni/directional case.

    Lattice basis: (1000, 0) and (500, 500*sqrt(3)).  The disk of radius 1800
    is covered by equilateral triangles of side 1000.  For every source G and
    every pointing direction, at least one vertex of the containing triangle
    lies both inside the 180-degree coverage half-plane and within 1000 m of G.
    Hence every directional source is detected as well.
    """
    h = 1000.0 * math.sqrt(3.0) / 2.0  # 866.025403784
    rows = [
        (-2598.0762113533, [-500.0, 500.0]),
        (-1732.0508075689, [-2000.0, -1000.0, 0.0, 1000.0, 2000.0]),
        (-866.0254037844, [-2500.0, -1500.0, -500.0, 500.0, 1500.0, 2500.0]),
        (0.0, [-2000.0, -1000.0, 0.0, 1000.0, 2000.0]),
        (866.0254037844, [-2500.0, -1500.0, -500.0, 500.0, 1500.0, 2500.0]),
        (1732.0508075689, [-2000.0, -1000.0, 0.0, 1000.0, 2000.0]),
        (2598.0762113533, [-500.0, 500.0]),
    ]
    pts = [(x, y) for y, xs in rows for x in xs]
    assert len(pts) == 31
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


def _scan_at(iface, point: tuple[float, float], data: dict[int, ChannelData]):
    """Measure all 20 channels at one point, starting from the current channel
    to avoid one avoidable switch."""
    cur = iface.current_channel
    order = [cur] + [c for c in CHANNELS if c != cur]
    for ch in order:
        res = iface.measure(point, ch)
        d = data[ch]
        if res.measure_result == "direction":
            d.signals.append((point, res.svd_deg))
            if d.first_signal_pos is None:
                d.first_signal_pos = point
        elif res.measure_result == "near":
            d.near_points.append(point)
            if d.first_signal_pos is None:
                d.first_signal_pos = point
        else:
            d.no_signal_points.append(point)


def survey(iface, points: list[tuple[float, float]],
           start: tuple[float, float] | None = None):
    """Visit `points`, scanning all 20 channels at each point.

    Returns dict channel -> ChannelData.
    """
    data = {ch: ChannelData() for ch in CHANNELS}
    if start is None:
        start = iface.pos
    route = geo.order_route(points, start)
    for p in route:
        _scan_at(iface, p, data)
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


def _cover_and_clear(iface, poly, channel: int, signals=None):
    """Guaranteed fallback: cover the convex feasible polygon by clear disks."""
    if not poly and signals:
        # Numeric clipping should never produce an empty polygon when signals
        # exist, but reconstruct the one-cone fallback if it does.
        poly = _rebuild_poly([signals[0]], [signals[0][0]])
    if not poly:
        return False
    pts = geo.cover_points_for_polygon(poly, CLEAR_RADIUS)
    if not pts:
        # Degenerate polygon: clear at its first vertex.
        pts = [poly[0]]
    route = geo.order_route(pts, iface.pos)
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
    poly = _rebuild_poly(signals, disk_centers)
    if not poly:
        return False

    for _ in range(MAX_Q4_ITER):
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
        # no_signal: either the point is outside the sector or (very unlikely
        # for a nearby center) outside range.  Use the guaranteed grid-clear
        # fallback on the conservative polygon.
        break

    return _cover_and_clear(iface, poly, channel, signals)


def clear_all(iface, mode: str, data: dict[int, ChannelData]) -> tuple[int, int]:
    """Clear all detected channels, using nearest-next-channel ordering."""
    active = [ch for ch in CHANNELS
              if data[ch].signals or data[ch].near_points]
    cleared = 0
    failed = 0
    remaining = set(active)
    while remaining:
        # Pick the channel whose next action point is nearest to the robot.
        best_ch = None
        best_point = None
        for ch in remaining:
            d = data[ch]
            if d.near_points:
                p = min(d.near_points, key=lambda q: math.dist(iface.pos, q))
            elif d.signals:
                poly = _rebuild_poly(d.signals, [p for p, _ in d.signals])
                if not poly:
                    p = d.signals[0][0]
                else:
                    p, _ = geo.min_enclosing_circle(poly)
            else:
                p = (0.0, 0.0)
            dist = math.dist(iface.pos, p)
            if best_ch is None or dist < math.dist(iface.pos, best_point):
                best_ch, best_point = ch, p
        remaining.discard(best_ch)
        ok = (clear_channel_q3(iface, best_ch, data[best_ch]) if mode == "q3"
              else clear_channel_q4(iface, best_ch, data[best_ch]))
        if ok:
            cleared += 1
        else:
            failed += 1
    return cleared, failed


def run_q3(iface) -> dict:
    pts = q3_survey_points()
    data = survey(iface, pts, iface.pos)
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
    data = survey(iface, pts, iface.pos)
    cleared, failed = clear_all(iface, "q4", data)
    return {
        "survey_points": pts,
        "data": data,
        "cleared": cleared,
        "failed": failed,
        "active_channels": [ch for ch in CHANNELS if data[ch].signals or data[ch].near_points],
    }
