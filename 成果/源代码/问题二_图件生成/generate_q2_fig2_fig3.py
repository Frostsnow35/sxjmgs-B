"""Generate Q2 Fig. 2 and Fig. 3 with auditable geometry.

Fig. 2: bounded bearing wedge and intersection of three wedges.
Fig. 3: equilateral-triangle counterexample for diameter-circle coverage.
All coordinates are in metres; angles are degrees.  These are construction
figures for the manuscript, not simulator observations.
"""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Polygon, Arc, FancyArrowPatch

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "figure"
OUT.mkdir(exist_ok=True)
plt.rcParams.update({
    "font.family": "Microsoft YaHei",
    "axes.unicode_minus": False,
    "font.size": 9,
    "figure.dpi": 180,
})

BLUE = "#173f5f"
MID = "#4f7f9f"
LIGHT = "#b9d2e3"
RED = "#b22222"
BLACK = "#111111"


def u(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([math.cos(t), math.sin(t)])


def wedge(origin: np.ndarray, theta: float, r: float = 1800.0, eps: float = 1.0, n: int = 80) -> np.ndarray:
    angles = np.radians(np.linspace(theta - eps, theta + eps, n))
    return np.vstack((origin, origin + r * np.c_[np.cos(angles), np.sin(angles)]))


def cross2(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def clip_halfplane(poly: np.ndarray, point: np.ndarray, direction: np.ndarray, left: bool = True) -> np.ndarray:
    if len(poly) == 0:
        return poly
    out: list[np.ndarray] = []
    for a, b in zip(poly, np.vstack((poly[1:], poly[:1]))):
        fa, fb = cross2(direction, a - point), cross2(direction, b - point)
        ia = fa >= -1e-10 if left else fa <= 1e-10
        ib = fb >= -1e-10 if left else fb <= 1e-10
        if ia:
            out.append(a)
        if ia != ib:
            out.append(a + fa / (fa - fb) * (b - a))
    return np.asarray(out)


def bearing_wedge_clip(poly: np.ndarray, sensor: np.ndarray, theta: float, eps: float = 1.0) -> np.ndarray:
    lo, hi = u(theta - eps), u(theta + eps)
    return clip_halfplane(clip_halfplane(poly, sensor, lo, True), sensor, hi, False)


def fig2() -> None:
    eps = 1.0
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 4.6), gridspec_kw={"width_ratios": [0.9, 1.1]})

    # (a) One sensor and one closed angular uncertainty wedge.
    ax = axes[0]
    s = np.array([0.0, 0.0]); theta = 35.0
    ax.add_patch(Polygon(wedge(s, theta), closed=True, fc=LIGHT, ec=MID, alpha=.48, lw=1.1))
    for ang, label, off in [(theta-eps, r"$\theta_i-1^\circ$", (25, -80)),
                            (theta+eps, r"$\theta_i+1^\circ$", (-145, 90))]:
        p = s + 1600 * u(ang)
        ax.plot([s[0], p[0]], [s[1], p[1]], color=BLUE, lw=1.0)
        ax.text(*(s + 1400*u(ang) + np.array(off)), label, color=BLACK, fontsize=9)
    p0 = s + 1450*u(theta)
    ax.plot([s[0], p0[0]], [s[1], p0[1]], color=BLUE, lw=.8, ls="--")
    ax.text(*(s + 960*u(theta) + np.array([15, 20])), r"$\theta_i$", color=BLUE, fontsize=9)
    ax.plot(*s, "o", ms=5, color=BLUE); ax.text(35, -75, r"$S_i=(x_i,y_i)$", color=BLUE)
    ax.annotate(r"$C_i$", xy=s + 1020*u(theta), xytext=(930, 570), color=BLUE,
                arrowprops={"arrowstyle": "->", "color": BLUE, "lw": .8})
    ax.set(title="(a) 单个检测点的闭角域", xlim=(-200, 1700), ylim=(-250, 1050), xlabel="$x$ / m", ylabel="$y$ / m")
    ax.set_aspect("equal"); ax.grid(alpha=.15)
    legend_a = [
        Line2D([0], [0], color=BLUE, lw=1.0, label="误差边界方向"),
        Line2D([0], [0], color=BLUE, lw=.8, ls="--", label="示向度中心线"),
        Polygon(np.empty((0, 2)), fc=LIGHT, ec=MID, alpha=.48, label=r"闭角域 $C_i$"),
    ]
    ax.legend(handles=legend_a, loc="upper left", fontsize=8, frameon=True)

    # (b) Three wedges; their half-plane intersection is shown as a polygon.
    ax = axes[1]
    truth = np.array([250.0, 260.0])
    sensors = [np.array([-950.0, -500.0]), np.array([1050.0, -480.0]), np.array([650.0, 950.0])]
    colors = ["#477a9e", "#9c6b32", "#6a4c93"]
    thetas = [math.degrees(math.atan2(*(truth-s)[::-1])) for s in sensors]
    for idx, (sensor, theta, col) in enumerate(zip(sensors, thetas, colors), start=1):
        ax.add_patch(Polygon(wedge(sensor, theta, 1700, eps), closed=True, fc=col, ec=col, alpha=.12, lw=1.0))
        ax.plot(*sensor, "s", color=col, ms=5)
        ax.text(*(sensor + np.array([35, -70])), rf"$S_{idx}$", color=col, fontsize=9)
        ax.plot([sensor[0], sensor[0]+1500*u(theta)[0]], [sensor[1], sensor[1]+1500*u(theta)[1]], color=col, lw=.9)
    poly = np.array([[-1700., -1700.], [1700., -1700.], [1700., 1700.], [-1700., 1700.]])
    for sensor, theta in zip(sensors, thetas):
        poly = bearing_wedge_clip(poly, sensor, theta, eps)
    ax.add_patch(Polygon(poly, closed=True, fc="#204f70", ec=BLUE, alpha=.65, lw=1.4, zorder=4))
    ax.plot(*truth, "o", color=RED, ms=4, zorder=6)
    ax.text(760, 640, "公共交集见右上局部放大", color=BLUE, fontsize=9, ha="center")
    ax.text(truth[0]+28, truth[1]+35, "$G$（示例点）", color=RED, fontsize=8)
    ax.set(title="(b) 多个测向楔形的公共交集", xlim=(-1500, 1500), ylim=(-1300, 1400), xlabel="$x$ / m", ylabel="$y$ / m")
    ax.set_aspect("equal"); ax.grid(alpha=.15)
    legend_b = [
        Line2D([0], [0], marker="s", color="w", markerfacecolor=colors[0], markersize=6, label="检测点 $S_i$"),
        Line2D([0], [0], color=colors[0], lw=1.0, label=r"测向边界（$±1^\circ$）"),
        Polygon(np.empty((0, 2)), fc="#204f70", ec=BLUE, alpha=.65, label=r"公共交集 $P=\bigcap_i C_i$"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor=RED, markersize=5, label="示例点 $G$（仅示意）"),
    ]
    ax.legend(handles=legend_b, loc="lower left", fontsize=8, frameon=True)
    fig.suptitle("图 2  有界测向误差下的楔形约束与公共交集", y=.995, fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, .96))
    for ext in ("png", "pdf", "svg"):
        fig.savefig(OUT / f"图2_测向楔形与交集.{ext}", dpi=320, bbox_inches="tight")
    plt.close(fig)


def fig2_compact() -> None:
    """Axis-free, single-panel geometry inspired by the team's reference figure."""
    eps = 1.0
    # A drawing magnification is used only for the translucent bands so that
    # the common polygon remains visible at page size.  The centre lines and
    # all numerical calculations retain the prescribed ±1° uncertainty.
    display_eps = 6.0
    fig, ax = plt.subplots(figsize=(7.4, 6.4))
    target = np.array([0.0, 0.0])
    # Shorter baselines keep the true ±1° uncertainty wedge and its intersection readable.
    sensors = [np.array([-100.0, -85.0]), np.array([112.0, -62.0]), np.array([28.0, 118.0])]
    colors = ["#28577a", "#8b5e2f", "#5c4b8a"]
    thetas = [math.degrees(math.atan2(-s[1], -s[0])) for s in sensors]
    poly = np.array([[-130., -130.], [130., -130.], [130., 130.], [-130., 130.]])
    for sensor, theta in zip(sensors, thetas):
        poly = bearing_wedge_clip(poly, sensor, theta, display_eps)
    for idx, (sensor, theta, col) in enumerate(zip(sensors, thetas, colors), start=1):
        ax.add_patch(Polygon(wedge(sensor, theta, 260, display_eps), closed=True, fc=col, ec="none", alpha=.12, zorder=1))
        for ang in (theta-display_eps, theta+display_eps):
            end = sensor + 250*u(ang)
            ax.add_patch(FancyArrowPatch(sensor, end, arrowstyle="-|>", mutation_scale=10, color=col, lw=1.05, zorder=2))
        mid = sensor + 190*u(theta)
        ax.plot([sensor[0], mid[0]], [sensor[1], mid[1]], color=col, lw=.8, ls=(0, (3, 3)), zorder=2)
        ax.plot(*sensor, "o", color=BLACK, ms=5, zorder=6)
        offsets = [(-26, -18), (8, -18), (8, 8)]
        off = np.array(offsets[idx-1])
        ax.text(*(sensor + off), rf"$S_{idx}$", color=BLACK, fontsize=10)
        ax.text(*(sensor + off + np.array([-3, -15])), rf"$(x_{idx},y_{idx})$", color=BLACK, fontsize=7.5)
        lp = sensor + 172*u(theta) + np.array([8, 8])
        ax.text(*lp, rf"$C_{idx}$", color=col, fontsize=10)
    ax.add_patch(Polygon(poly, closed=True, fc="#6ea5ce", ec=BLUE, alpha=.82, lw=1.5, zorder=4))
    # Keep the main panel uncluttered: the filled polygon itself is the
    # indication of the common feasible region.  A short leader places its
    # label away from the crossing rays.
    ax.annotate(r"$P=\bigcap_i C_i$", xy=target, xytext=(-45, 42), color=BLUE,
                fontsize=10, ha="right", va="bottom",
                bbox={"facecolor": "white", "edgecolor": "none", "pad": 1.5},
                arrowprops={"arrowstyle": "->", "color": BLUE, "lw": .8}, zorder=9)
    handles = [
        Line2D([0], [0], color=BLUE, lw=1.0, label=r"误差边界（方向按 $\pm1^\circ$）"),
        Line2D([0], [0], color=BLUE, lw=.8, ls=(0, (3, 3)), label="示向度中心线"),
        Polygon(np.empty((0, 2)), fc="#6ea5ce", ec=BLUE, alpha=.82, label=r"公共交集 $P=\bigcap_i C_i$"),
    ]
    ax.legend(handles=handles, loc="upper left", fontsize=8, frameon=True, borderpad=.6)
    ax.text(.5, .025, r"楔形宽度为视觉放大示意；实际误差按 $\pm1^\circ$ 计算，公共部分为定位区域 $P$",
            transform=ax.transAxes,
            ha="center", va="bottom", fontsize=9, color="#333333")
    ax.set_xlim(-155, 165); ax.set_ylim(-145, 160); ax.set_aspect("equal"); ax.axis("off")
    fig.tight_layout(pad=.5)
    for ext in ("png", "pdf", "svg"):
        fig.savefig(OUT / f"图2_测向楔形与交集_紧凑版.{ext}", dpi=320, bbox_inches="tight")
    plt.close(fig)


def fig3() -> None:
    D = 2.0
    A, B, C = np.array([-1., 0.]), np.array([1., 0.]), np.array([0., math.sqrt(3)])
    O = (A + B) / 2
    fig, ax = plt.subplots(figsize=(6.6, 5.2))
    ax.add_patch(Polygon(np.vstack((A, B, C)), closed=True, fc="#d9e7f0", ec=BLUE, lw=1.35, alpha=.85))
    ax.add_patch(Circle(O, D/2, fill=False, ec=RED, lw=1.35, ls="--"))
    ax.plot([A[0], B[0]], [A[1], B[1]], color=RED, lw=2.0, label=r"直径端点 $AB$（$D=2$ m）")
    ax.plot([O[0], C[0]], [O[1], C[1]], color=BLACK, lw=.95, ls=":")
    ax.plot([O[0], A[0]], [O[1], A[1]], color=BLACK, lw=.8, ls="--")
    ax.plot([O[0], B[0]], [O[1], B[1]], color=BLACK, lw=.8, ls="--")
    for p, label, off, col in [(A, "$A$", (-.12,-.22), BLUE), (B, "$B$", (.08,-.22), BLUE),
                                (C, "$C$", (.08,.05), BLUE), (O, "$O$", (.06,-.20), BLACK)]:
        ax.plot(*p, "o", color=col, ms=5, zorder=5); ax.text(*(p+off), label, color=col, fontsize=10)
    # Place explanatory labels in the open right-hand margin; leaders point
    # to the corresponding construction without crossing text or vertices.
    ax.annotate(r"$OC=\sqrt{3}\,\mathrm{m}>D/2=1\,\mathrm{m}$",
                xy=(0.0, 0.98), xytext=(0.62, 1.78),
                arrowprops={"arrowstyle": "->", "lw": .8, "color": BLACK},
                fontsize=9, ha="left", va="center",
                bbox={"facecolor": "white", "edgecolor": "none", "pad": 1.5})
    ax.annotate(r"以 $AB$ 为直径的圆",
                xy=(0.74, 0.67), xytext=(1.18, 1.48),
                arrowprops={"arrowstyle": "->", "color": RED, "lw": .8},
                color=RED, fontsize=9, ha="left", va="center",
                bbox={"facecolor": "white", "edgecolor": "none", "pad": 1.5})
    ax.text(-.82, -.26, r"$AO=BO=D/2=1\,m$", fontsize=8, color=BLACK)
    # 图题由论文正文统一添加，图内不重复设置标题。
    ax.set(title=None, xlim=(-1.55, 1.65), ylim=(-.42, 2.03), xlabel="$x$ / m", ylabel="$y$ / m")
    ax.set_aspect("equal"); ax.grid(alpha=.15)
    ax.legend(loc="upper left", fontsize=8, frameon=True)
    fig.tight_layout()
    for ext in ("png", "pdf", "svg"):
        fig.savefig(OUT / f"图3_等边三角形直径圆反例.{ext}", dpi=320, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    fig2(); fig2_compact(); fig3(); print(f"generated in {OUT}")
