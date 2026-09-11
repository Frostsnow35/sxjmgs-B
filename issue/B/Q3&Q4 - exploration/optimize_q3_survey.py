"""Search for a six-outer-point Q3 survey configuration with shorter route.

Keeps the centre point fixed.  A configuration is feasible when the largest
distance from any arena point to its nearest survey point is <= 1000 m.
The objective is the exact TSP route length from the origin through all seven
points.  This is only a numerical search; the regular hexagon is used as the
safety-default start and fallback.
"""
from __future__ import annotations

import math

import numpy as np
from scipy.optimize import differential_evolution

import strategy
import geometry as geo

ARENA_R = 1800.0
COVER_R = 1000.0
COVER_LIMIT = float(__import__("os").environ.get("Q3_COVER_LIMIT", "1000.0"))

TH = np.linspace(0.0, 2.0 * math.pi, 180, endpoint=False)
RR = np.linspace(0.0, ARENA_R, 36)
GRID = np.array([[r * math.cos(t), r * math.sin(t)]
                 for t in TH for r in RR])
GRID = GRID[np.sum(GRID * GRID, axis=1) <= ARENA_R * ARENA_R + 1.0]


def points_from_vector(z):
    return [(0.0, 0.0)] + [(float(z[2 * i]), float(z[2 * i + 1]))
                           for i in range(6)]


def max_nearest(z):
    pts = points_from_vector(z)
    arr = np.array(pts[1:], dtype=float)
    d2 = np.sum((GRID[:, None, :] - arr[None, :, :]) ** 2, axis=2)
    d2 = np.minimum(d2, np.sum(GRID * GRID, axis=1, keepdims=True))
    return float(np.sqrt(np.max(np.min(d2, axis=1))))


def route_length(z):
    pts = points_from_vector(z)
    ordered = geo.order_points_exact_tsp(pts, (0.0, 0.0))
    return geo.route_length(ordered, (0.0, 0.0))


def objective(z):
    cover = max_nearest(z)
    penalty = 0.0
    if cover > COVER_LIMIT:
        penalty = 1e6 * (cover - COVER_LIMIT) ** 2 + 1000.0 * (cover - COVER_LIMIT)
    return route_length(z) + penalty


def main():
    # Default: regular hexagon.
    a = 900.0 * math.sqrt(3.0)
    x0 = []
    for k in range(6):
        t = math.pi / 3.0 * k
        x0 += [a * math.cos(t), a * math.sin(t)]
    print("default route", route_length(np.array(x0)),
          "max_nearest", max_nearest(np.array(x0)))
    bounds = [(-2300.0, 2300.0)] * 12
    result = differential_evolution(
        objective, bounds, seed=7, maxiter=80, popsize=8,
        tol=1e-5, polish=True, disp=False, workers=1)
    print("best route", route_length(result.x),
          "max_nearest", max_nearest(result.x))
    print("points", points_from_vector(result.x))


if __name__ == "__main__":
    main()
