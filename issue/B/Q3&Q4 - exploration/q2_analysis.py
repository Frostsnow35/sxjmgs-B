"""Q2 numerical exploration: where should the second detection point be?

First measurement point P1 = (0,0), returned bearing 0 degrees.  The true
source G must satisfy

    angle(P1 -> G) in [-1, 1] deg,   |G - P1| <= 1500 m

(the last inequality follows because a detected omni source has effective
radius <= 1500 m).

For a candidate second point P2 we know the true bearing from P2 to G has the
same +/-1 degree error bound.  We evaluate, over a grid of possible true source
positions, the radius of the smallest circle containing the feasible polygon

    F = cone(P1, 0, 1) ∩ cone(P2, a2(G), 1) ∩ disk(P1, 1500).

Smaller radius means better localization.
"""
from __future__ import annotations

import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import geometry as geo

P1 = (0.0, 0.0)
RMAX = 1500.0
T_GRID = list(range(20, 1501, 100)) + [1499]
D_GRID = [-1.0, -0.5, 0.0, 0.5, 1.0]
_BASE_HPS = (geo.wedge_halfplanes(P1, 0.0) +
             geo.disk_halfplanes(P1, RMAX, n=120))


def true_source_points():
    pts = []
    for t in T_GRID:
        for d in D_GRID:
            rad = math.radians(d)
            pts.append((t * math.cos(rad), t * math.sin(rad)))
    return pts


_G = true_source_points()


def worst_radius_for_p2(p2: tuple[float, float]) -> tuple[float, float]:
    """(worst MEC radius, max distance from P2 to a possible source)."""
    worst = 0.0
    max_dist = 0.0
    for g in _G:
        max_dist = max(max_dist, math.dist(p2, g))
        a2 = math.degrees(math.atan2(g[1] - p2[1], g[0] - p2[0]))
        poly = geo.polygon_from_halfplanes(_BASE_HPS + geo.wedge_halfplanes(p2, a2))
        if not poly:
            continue
        _, rad = geo.min_enclosing_circle(poly)
        worst = max(worst, rad)
    return worst, max_dist


def main():
    # Two evaluations:
    # A) all candidates; record worst localization radius.
    # B) only candidates that are guaranteed to receive signal for every
    #    possible source: max distance to every G in first cone <= 1000.
    rr = np.linspace(100.0, 1900.0, 19)
    pp = np.linspace(0.0, 90.0, 31)
    best_all = None
    best_safe = None
    matrix = np.full((len(rr), len(pp)), np.nan)
    safe_matrix = np.full((len(rr), len(pp)), np.nan)
    for i, r in enumerate(rr):
        for j, ph in enumerate(pp):
            p2 = (r * math.cos(math.radians(ph)), r * math.sin(math.radians(ph)))
            wr, maxd = worst_radius_for_p2(p2)
            matrix[i, j] = wr
            if maxd <= 1000.0 + 1e-6:
                safe_matrix[i, j] = wr
                if best_safe is None or wr < best_safe[0]:
                    best_safe = (wr, r, ph)
            if best_all is None or wr < best_all[0]:
                best_all = (wr, r, ph)

    print("=== Q2 numerical summary ===")
    print(f"best unconstrained P2: R={best_all[1]:.1f} m, "
          f"phi={best_all[2]:.1f} deg, worst MEC radius={best_all[0]:.1f} m")
    print(f"best signal-guaranteed P2: R={best_safe[1]:.1f} m, "
          f"phi={best_safe[2]:.1f} deg, worst MEC radius={best_safe[0]:.1f} m")

    # Candidate region from the analysis: annular sector(s), symmetric with
    # respect to the first bearing line.  The two lobes are phi in [30,60] or
    # [-60,-30], R in [1200,1800] if a second signal is not mandatory; the
    # guaranteed-signal lobe is narrower and near R=1000.
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    for ax, mat, title in [
        (axes[0], matrix, "Worst MEC radius (m), all candidates"),
        (axes[1], safe_matrix, "Worst MEC radius (m), guaranteed 2nd signal"),
    ]:
        im = ax.pcolormesh(pp, rr, mat, shading="auto", cmap="viridis_r")
        ax.set_xlabel("angular offset of P2 from first bearing (deg)")
        ax.set_ylabel("distance P1 -> P2 (m)")
        ax.set_title(title)
        fig.colorbar(im, ax=ax)
    fig.tight_layout()
    out = Path("q2_candidate_heatmap.png")
    fig.savefig(out, dpi=130)
    print("saved", out)

    # Candidate region description.
    print("""
Recommended second-point candidate region (symmetric on both sides of the
measured bearing):
  R = |P2-P1| in [1200, 1800] m and angular offset 30-60 degrees from the
  first bearing.  Best localisation in the tested grid is near R=1500 m and
  offset 45 degrees.
If we also require that the second point is certain to receive the signal for
every source consistent with the first measurement (d(P2,G) <= 1000 m), the
candidate region shrinks to R <= 1000 m with an angular offset up to about
40 degrees; its best point is near R=1000 m and offset 32-35 degrees.
""")


if __name__ == "__main__":
    main()
