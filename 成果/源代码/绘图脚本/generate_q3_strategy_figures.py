"""Reproducible Q3 explanatory figures.

The route is a deterministic synthetic scenario, not a simulator drill or a
formal-test result.  It is deliberately separated from JLOG-derived evidence.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon

ROOT = Path(__file__).resolve().parent
FIG = ROOT / "figures"
DATA = ROOT / "figure_data"
FIG.mkdir(exist_ok=True)
DATA.mkdir(exist_ok=True)

AREA_R = 1800.0
RECV_R = 1500.0
EPS = 1.02
CLEAR_R = 20.0

plt.rcParams.update({"font.family": "Microsoft YaHei", "axes.unicode_minus": False,
                     "font.size": 9, "figure.dpi": 180})


def bearing(s: np.ndarray, g: np.ndarray) -> float:
    return math.degrees(math.atan2(g[1] - s[1], g[0] - s[0]))


def circle_poly(center: np.ndarray, radius: float, n: int = 160) -> np.ndarray:
    t = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    return center + radius * np.c_[np.cos(t), np.sin(t)]


def clip_halfplane(poly: np.ndarray, p: np.ndarray, direction: np.ndarray, keep_left: bool) -> np.ndarray:
    """Clip a convex polygon to one directed-line half plane."""
    if len(poly) == 0:
        return poly
    out: list[np.ndarray] = []
    for a, b in zip(poly, np.vstack((poly[1:], poly[:1]))):
        fa = direction[0] * (a[1] - p[1]) - direction[1] * (a[0] - p[0])
        fb = direction[0] * (b[1] - p[1]) - direction[1] * (b[0] - p[0])
        ia = fa >= -1e-10 if keep_left else fa <= 1e-10
        ib = fb >= -1e-10 if keep_left else fb <= 1e-10
        if ia:
            out.append(a)
        if ia != ib:
            out.append(a + fa / (fa - fb) * (b - a))
    return np.asarray(out)


def clip_convex(subject: np.ndarray, clipper: np.ndarray) -> np.ndarray:
    """Sutherland--Hodgman clipping by a counter-clockwise convex polygon."""
    out = subject
    for p, q in zip(clipper, np.vstack((clipper[1:], clipper[:1]))):
        out = clip_halfplane(out, p, q - p, True)
        if len(out) == 0:
            break
    return out


def wedge_clip(poly: np.ndarray, sensor: np.ndarray, measured_deg: float) -> np.ndarray:
    lo = math.radians(measured_deg - EPS)
    hi = math.radians(measured_deg + EPS)
    u_lo = np.array([math.cos(lo), math.sin(lo)])
    u_hi = np.array([math.cos(hi), math.sin(hi)])
    return clip_halfplane(clip_halfplane(poly, sensor, u_lo, True), sensor, u_hi, False)


def mec(points: np.ndarray) -> tuple[np.ndarray, float]:
    """Deterministic small-set MEC suitable for a convex polygon vertex set."""
    pts = [np.asarray(p, dtype=float) for p in points]
    best: tuple[np.ndarray, float] | None = None
    candidates: list[tuple[np.ndarray, float]] = [(p, 0.0) for p in pts]
    for i in range(len(pts)):
        for j in range(i):
            c = (pts[i] + pts[j]) / 2.0
            candidates.append((c, float(np.linalg.norm(pts[i] - c))))
    for i in range(len(pts)):
        for j in range(i):
            for k in range(j):
                a, b, c0 = pts[i], pts[j], pts[k]
                d = 2.0 * ((b[0]-a[0])*(c0[1]-a[1]) - (b[1]-a[1])*(c0[0]-a[0]))
                if abs(d) < 1e-9:
                    continue
                aa, bb, cc = np.dot(a, a), np.dot(b, b), np.dot(c0, c0)
                x = (aa * (b[1]-c0[1]) + bb * (c0[1]-a[1]) + cc * (a[1]-b[1])) / d
                y = (aa * (c0[0]-b[0]) + bb * (a[0]-c0[0]) + cc * (b[0]-a[0])) / d
                center = np.array([x, y])
                candidates.append((center, float(np.linalg.norm(center-a))))
    for center, radius in candidates:
        if np.all(np.linalg.norm(points - center, axis=1) <= radius + 1e-6):
            if best is None or radius < best[1]:
                best = (center, radius)
    if best is None:
        raise RuntimeError("MEC construction failed")
    return best


def feasible_polygon(observations: Iterable[tuple[np.ndarray, float]]) -> np.ndarray:
    poly = circle_poly(np.zeros(2), AREA_R)
    for sensor, theta in observations:
        poly = wedge_clip(poly, sensor, theta)
        poly = clip_convex(poly, circle_poly(sensor, RECV_R, 128))
    return poly


def plot_feasible_mec() -> dict[str, float]:
    """Fig. 9: global constraints -> local intersection -> MEC decision."""
    truth = np.array([450.0, 350.0])
    sensors = [np.array([-800.0, -450.0]), np.array([500.0, -700.0]), np.array([1250.0, 300.0])]
    noise = [0.60, -0.55, 0.18]
    obs = [(s, bearing(s, truth) + e) for s, e in zip(sensors, noise)]
    final_poly = feasible_polygon(obs)
    center, radius = mec(final_poly)
    fig, axes = plt.subplots(1, 3, figsize=(12.0, 3.85), gridspec_kw={"width_ratios": [1.02, 1.04, .94]})
    # (a) Global geometry: only the constraints and the final-region callout.
    ax = axes[0]
    ax.add_patch(Circle((0, 0), AREA_R, fill=False, ec="#555555", lw=.9, ls="--"))
    for idx, (sensor, theta) in enumerate(obs, start=1):
        u = np.array([math.cos(math.radians(theta)), math.sin(math.radians(theta))])
        ax.plot([sensor[0], sensor[0]+2200*u[0]], [sensor[1], sensor[1]+2200*u[1]], color="#6e91aa", lw=.85)
        ax.plot(*sensor, "s", color="#173f5f", ms=4)
        ax.text(sensor[0]+45, sensor[1]-85, f"$S_{idx}$", color="#173f5f", fontsize=9)
    minxy, maxxy = final_poly.min(axis=0), final_poly.max(axis=0)
    margin = 45
    ax.add_patch(Polygon(final_poly, closed=True, fc="#1f77b4", ec="#173f5f", alpha=.7, lw=1.0, zorder=4))
    ax.add_patch(plt.Rectangle(minxy-margin, *(maxxy-minxy+2*margin), fill=False, ec="#b22222", lw=1.0, ls="-"))
    ax.annotate("局部放大区域", xy=(maxxy[0], maxxy[1]), xytext=(-1450, 1450), color="#b22222", fontsize=8,
                arrowprops={"arrowstyle": "->", "color": "#b22222", "lw": .8})
    ax.set(title="(a) 多点测向的全局几何", xlim=(-1900,1900), ylim=(-1900,1900), xlabel="$x$ / m", ylabel="$y$ / m")
    ax.set_aspect("equal"); ax.grid(alpha=.13)
    # (b) Zoom: render only local wedge boundaries and the active receiver arcs.
    ax = axes[1]
    for sensor, theta in obs:
        t = np.linspace(math.radians(theta-EPS), math.radians(theta+EPS), 100)
        ax.fill(np.r_[sensor[0], sensor[0]+2800*np.cos(t)], np.r_[sensor[1], sensor[1]+2800*np.sin(t)], color="#6e91aa", alpha=.10)
        ax.add_patch(Circle(sensor, RECV_R, fill=False, ec="#9aa8b5", lw=.75, ls=":"))
    ax.add_patch(Polygon(final_poly, closed=True, fc="#1f77b4", ec="#173f5f", alpha=.62, lw=1.25, zorder=4))
    local_pad = 110
    ax.set(title=r"(b) 局部约束取交得到 $\hat F_c$", xlim=(minxy[0]-local_pad,maxxy[0]+local_pad), ylim=(minxy[1]-local_pad,maxxy[1]+local_pad), xlabel="$x$ / m")
    ax.set_aspect("equal"); ax.grid(alpha=.15)
    ax.text(.03,.04,"浅蓝：角域；灰虚线：接收约束边界", transform=ax.transAxes, fontsize=7,
            bbox={"fc":"white", "ec":"none", "alpha":.9})
    # (c) Decision-scale view: no truth point, only computable envelope objects.
    ax = axes[2]
    pad = 29
    ax.add_patch(Polygon(final_poly, closed=True, fc="#1f77b4", ec="#173f5f", alpha=.55, lw=1.15, zorder=3))
    ax.add_patch(Circle(center, CLEAR_R, fill=False, ec="#111111", lw=1.0, ls="--", zorder=5, label="20 m 清除圆"))
    ax.add_patch(Circle(center, radius, fill=False, ec="#b22222", lw=1.55, zorder=6, label="MEC"))
    ax.plot(*center, marker="*", color="#b22222", ms=10, zorder=7)
    ax.annotate(r"$m_c$", xy=center, xytext=(center[0]+16,center[1]+17), fontsize=9,
                arrowprops={"arrowstyle":"->", "lw":.75})
    ax.set(title=r"(c) MEC 包络与清除判据", xlim=(center[0]-pad,center[0]+pad), ylim=(center[1]-pad,center[1]+pad), xlabel="$x$ / m")
    ax.set_aspect("equal"); ax.grid(alpha=.15); ax.legend(loc="upper left", fontsize=7, frameon=True)
    ax.text(.04,.04, r"$R_c=%.2f\,\mathrm{m}<19\,\mathrm{m}<20\,\mathrm{m}$" % radius,
            transform=ax.transAxes, fontsize=8, bbox={"fc":"white","ec":"none","alpha":.9})
    fig.tight_layout()
    for ext in ("png", "pdf", "svg"):
        fig.savefig(FIG / f"q3_feasible_mec_contraction.{ext}", dpi=320, bbox_inches="tight")
    plt.close(fig)
    return {"mec_radius_m": round(radius, 4)}


def plot_synthetic_route() -> dict[str, object]:
    """Fig. 10: deterministic synthetic execution trace following Q3 policy."""
    anchors = [np.array([0.0, 0.0])] + [1000*np.array([math.cos(2*math.pi*k/7), math.sin(2*math.pi*k/7)]) for k in range(7)]
    sources = [
        ("C01", (1130, 250), 1230), ("C02", (560, 1050), 1180), ("C03", (-100, 1370), 1200),
        ("C04", (-1050, 950), 1300), ("C05", (-1300, 200), 1160), ("C06", (-950, -800), 1250),
        ("C07", (-150, -1450), 1280), ("C08", (650, -1150), 1220), ("C09", (1420, -500), 1300),
        ("C10", (300, 450), 1150),
    ]
    targets = [(name, np.array(xy, dtype=float), float(r)) for name, xy, r in sources]
    path: list[np.ndarray] = [anchors[0]]; scan_idx = [0]; safe_idx: list[int] = []; clear_idx: list[int] = []
    discovered: set[str] = set(); current = anchors[0]
    offsets = [np.array([7.0, -5.0]), np.array([-6.0, 8.0]), np.array([5.0, 6.0]), np.array([-8.0, -4.0])]
    for i, anchor in enumerate(anchors):
        if i > 0:
            path.append(anchor); scan_idx.append(len(path)-1); current = anchor
        newly = [(name, g, rr) for name, g, rr in targets if name not in discovered and np.linalg.norm(g-anchor) <= rr]
        for j, (name, g, _) in enumerate(newly):
            discovered.add(name)
            u = (g-anchor) / np.linalg.norm(g-anchor); v = np.array([-u[1], u[0]])
            p_plus, p_minus = anchor + 600*u + 250*v, anchor + 600*u - 250*v
            second = p_plus if np.linalg.norm(current-p_plus) <= np.linalg.norm(current-p_minus) else p_minus
            path.append(second); safe_idx.append(len(path)-1)
            clear = g + offsets[(i+j) % len(offsets)]
            path.append(clear); clear_idx.append(len(path)-1); current = clear
    # Continue only if any synthetic source was not received by the coverage scan.
    assert len(discovered) == len(targets), "synthetic radii must satisfy full discovery"
    p = np.vstack(path)
    length = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
    fig, ax = plt.subplots(figsize=(8.1, 7.1))
    ax.add_patch(Circle((0, 0), AREA_R, fc="#f7f7f7", ec="#111111", lw=1.25))
    # baseline certified anchors and actual action trajectory are visually distinct.
    ring = np.vstack(anchors[1:] + [anchors[1]])
    ax.plot(ring[:,0], ring[:,1], ls="--", lw=.85, color="#9b9b9b", label="认证覆盖环（基准）")
    ax.plot(p[:,0], p[:,1], color="#173f5f", lw=1.35, zorder=3, label="合成算例的滚动动作轨迹")
    for a, b in zip(p[:-1], p[1:]):
        ax.annotate("", xy=b, xytext=a, arrowprops={"arrowstyle": "->", "color": "#173f5f", "lw": .85, "shrinkA": 5, "shrinkB": 5})
    aa = np.vstack(anchors)
    ax.scatter(aa[:,0], aa[:,1], marker="s", s=28, fc="white", ec="#111111", zorder=5, label="覆盖巡测点")
    ax.scatter(p[safe_idx,0], p[safe_idx,1], marker="^", s=36, color="#c66b17", zorder=6, label="安全二测点")
    ax.scatter(p[clear_idx,0], p[clear_idx,1], marker="*", s=70, color="#b22222", zorder=7, label="清除动作点（定位误差≤10 m）")
    gs = np.vstack([g for _, g, _ in targets])
    ax.scatter(gs[:,0], gs[:,1], marker="x", s=38, color="#555555", zorder=4, label="合成源真实位置（仅评估用）")
    for i, a in enumerate(anchors):
        ax.text(a[0]+35, a[1]+35, f"A{i}", fontsize=8, color="#333333")
    ax.text(-1710, 1575, "合成算例：10 个全向源；非演练、非正式测试", fontsize=9,
            bbox={"fc": "white", "ec": "#777777", "lw": .6, "alpha": .92})
    ax.set(xlim=(-1900,1900), ylim=(-1900,1900), xlabel="$x$ / m", ylabel="$y$ / m")
    ax.set_aspect("equal"); ax.grid(alpha=.15); ax.legend(loc="lower right", fontsize=8, frameon=True)
    fig.tight_layout()
    for ext in ("png", "pdf", "svg"):
        fig.savefig(FIG / f"q3_synthetic_rolling_route.{ext}", dpi=320, bbox_inches="tight")
    plt.close(fig)
    return {"source_count": len(targets), "anchor_count": len(anchors), "path_waypoint_count": len(path),
            "path_length_m": round(length, 3), "sources": [{"channel": n, "position_m": g.tolist(), "effective_radius_m": r} for n,g,r in targets]}


if __name__ == "__main__":
    result = {"figure_9": plot_feasible_mec(), "figure_10": plot_synthetic_route(),
              "provenance": "deterministic synthetic explanatory scenario; not simulator data"}
    (DATA / "q3_synthetic_figure_scenario.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
