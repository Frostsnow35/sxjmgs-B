"""Verify the survey designs and draw the survey-point maps."""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import strategy
import geometry as geo

ARENA_R = 1800.0


def check_q3():
    pts = strategy.q3_survey_points()
    worst = 0.0
    worst_at = None
    for th in np.linspace(0, 2 * math.pi, 721):
        for r in np.linspace(0, ARENA_R, 73):
            g = np.array([r * math.cos(th), r * math.sin(th)])
            d = min(math.dist(g, p) for p in pts)
            if d > worst:
                worst = d
                worst_at = g
    print("Q3 survey points:", len(pts))
    print("max nearest-survey-point distance over arena disk:", worst,
          "at", worst_at)
    print("guaranteed detection margin:", 1000 - worst, "m")
    return pts


def check_q4():
    pts = strategy.q4_survey_points()
    print("Q4 survey points:", len(pts))
    # 1) 有限格网证书：目标圆盘包含于顶点凸包；所有与圆盘相交的
    # Delaunay 三角形的最长边均不超过 1000m。二者合用即可说明任意
    # 源点属于一个三个顶点都不超过 1000m 的保留三角形。
    from scipy.spatial import ConvexHull, Delaunay
    arr = np.array(pts, dtype=float)
    tri = Delaunay(arr)
    hull = ConvexHull(arr)
    hull_inradius = min(
        abs(row[2]) / math.hypot(row[0], row[1])
        for row in hull.equations
    )
    relevant_triangles = []
    for simplex in tri.simplices:
        vertices = [tuple(arr[index]) for index in simplex]
        if geo.point_to_polygon_distance((0.0, 0.0), vertices) <= ARENA_R + 1e-8:
            longest_edge = max(
                math.dist(vertices[index], vertices[(index + 1) % 3])
                for index in range(3)
            )
            relevant_triangles.append(longest_edge)
    print("convex-hull inradius:", hull_inradius)
    print("target-intersecting triangles:", len(relevant_triangles),
          "max edge:", max(relevant_triangles, default=float("inf")))
    assert hull_inradius >= ARENA_R - 1e-8
    assert relevant_triangles and max(relevant_triangles) <= 1000.0 + 1e-8
    # 2) Direct half-disk hitting test on a reasonably dense grid, using
    #    vectorization.  For every (G, phi) check existence of a survey point
    #    inside the half-disk of radius 1000.
    rng_g = []
    for th in np.linspace(0, 2 * math.pi, 240, endpoint=False):
        for r in np.linspace(0, ARENA_R, 25):
            rng_g.append((r * math.cos(th), r * math.sin(th)))
    G = np.array(rng_g, dtype=float)
    phis = np.linspace(0, 2 * math.pi, 240, endpoint=False)
    U = np.column_stack([np.cos(phis), np.sin(phis)])
    P = np.array(pts, dtype=float)
    bad = []
    worst_hit_distance = 0.0
    # Loop over G (6000) and phi (240), but vectorize over points.
    for g in G:
        d = P - g
        dist = np.linalg.norm(d, axis=1)
        within = dist <= 1000.0 + 1e-8
        if not within.any():
            bad.append(("no point within 1000", g))
            continue
        for u in phis:
            uu = np.array([math.cos(u), math.sin(u)])
            ok = within & (d @ uu >= -1e-8)
            if not ok.any():
                bad.append(("half-disk miss", g, u))
                break
        if bad:
            break
    print("half-disk hitting failures:", len(bad))
    if bad:
        print("first failure:", bad[0])
    else:
        print("every tested directional half-disk contains a survey point")
    return pts


def plot_points(pts, name):
    fig, ax = plt.subplots(figsize=(7, 7))
    th = np.linspace(0, 2 * math.pi, 400)
    ax.plot(ARENA_R * np.cos(th), ARENA_R * np.sin(th), "k--", lw=1)
    for x, y in pts:
        ax.plot(x, y, "ro", ms=4)
        ax.add_patch(plt.Circle((x, y), 20.0, color="red", alpha=0.10))
    ax.set_aspect("equal", "box")
    ax.set_title(name)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.3)
    out = Path(name + ".png")
    fig.savefig(out, dpi=130)
    print("saved", out)
    plt.close(fig)


if __name__ == "__main__":
    q3 = check_q3()
    q4 = check_q4()
    plot_points(q3, "q3_survey_points")
    plot_points(q4, "q4_survey_points")
