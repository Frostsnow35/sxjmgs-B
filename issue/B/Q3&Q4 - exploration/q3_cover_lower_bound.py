"""Numerical evidence on whether fewer than 7 points can cover the radius-1800
disk with disks of radius 1000."""
from __future__ import annotations

import math

import numpy as np
from scipy.optimize import differential_evolution

R_ARENA = 1800.0
R_COVER = 1000.0

# Sample the arena disk; objective = maximum distance to nearest center.
TH = np.linspace(0.0, 2.0 * math.pi, 360, endpoint=False)
RR = np.linspace(0.0, R_ARENA, 60)
GRID = np.array([[r * math.cos(t), r * math.sin(t)]
                 for t in TH for r in RR])
GRID = GRID[np.sum(GRID * GRID, axis=1) <= R_ARENA * R_ARENA + 1.0]


def worst_radius(centers: np.ndarray) -> float:
    c = centers.reshape(-1, 2)
    d2 = np.sum((GRID[:, None, :] - c[None, :, :]) ** 2, axis=2)
    return float(np.sqrt(np.max(np.min(d2, axis=1))))


def main():
    for n in [4, 5, 6, 7]:
        bounds = [(-2200, 2200)] * (2 * n)
        t0 = math.inf
        for seed in range(2):
            res = differential_evolution(
                lambda z: worst_radius(z), bounds,
                maxiter=300, popsize=12, tol=1e-7, polish=True,
                seed=100 + seed, workers=1)
            if res.fun < t0:
                t0 = res.fun
                best = res.x
        print(f"n={n}: best found covering radius = {t0:.3f} m; "
              f"can cover with 1000m? {t0 <= R_COVER + 1e-6}")
        print("  centers:", np.round(best.reshape(-1, 2), 1))


if __name__ == "__main__":
    main()
