"""Run reproducible, offline validation cases for CUMCM 2026 B questions 1 and 2.

This entry point intentionally never imports or calls the simulator.  The cases
are analytic constructions that validate geometry and scoring semantics before
any future simulator-authorized work.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import logging
from pathlib import Path
import platform
import sys
from typing import Any

# Direct execution sets sys.path[0] to scripts/, whereas pytest imports from the
# repository root.  Make the documented direct command resolve the shared src/
# package without requiring callers to set PYTHONPATH.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.localization_geometry import (
    LocalizationResult,
    diameter_circle_covers,
    locate_from_bearings,
    rank_second_points,
    sample_first_direction_region,
    sample_target_disk,
    strict_second_point_kernel,
)


LOGGER = logging.getLogger(__name__)


def build_validation_report() -> dict[str, Any]:
    """Build the structured validation evidence for questions 1 and 2.

    No measurement instances are fabricated: all values below are labelled
    constructed cases with inputs included in the output JSON.
    """
    exact_observations = ((0.0, 0.0, 0.0), (1.0, 0.0, 90.0))
    exact_result = locate_from_bearings(exact_observations, error_deg=0.0)

    wrapped_observations = ((0.0, 0.0, 359.0), (1.0, -1.0, 90.0))
    wrapped_result = locate_from_bearings(wrapped_observations, error_deg=2.0)

    unbounded_observations = ((0.0, 0.0, 359.0),)
    unbounded_result = locate_from_bearings(
        unbounded_observations,
        error_deg=1.0,
        target_radius_m=None,
    )

    equilateral_vertices = ((0.0, 0.0), (2.0, 0.0), (1.0, 3.0**0.5))
    triangle_covers, triangle_diameter_m, triangle_centers = diameter_circle_covers(equilateral_vertices)

    first_point = (0.0, 0.0)
    direction_region_grid = sample_first_direction_region(
        first_point=first_point,
        bearing_deg=359.0,
        grid_spacing_m=100.0,
        error_deg=1.0,
        target_radius_m=1_800.0,
        maximum_receive_radius_m=1_500.0,
    )
    source_samples = ((100.0, 0.0), (200.0, 0.0))
    q2_ranked = rank_second_points(
        first_point=first_point,
        source_points=source_samples,
        candidates=((300.0, 0.0), (0.0, 100.0)),
        guaranteed_receive_radius_m=1_000.0,
    )
    q2_strict_kernel = strict_second_point_kernel(
        first_point=first_point,
        source_points=source_samples,
        candidates=((300.0, 0.0), (0.0, 100.0)),
        angle_tolerance_deg=70.0,
        guaranteed_receive_radius_m=1_000.0,
    )
    q2_refinement = _grid_refinement(first_point)

    return {
        "schema_version": "1.0",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "offline_constructed_validation_cases_only",
        "simulator_called": False,
        "runtime": {"python": platform.python_version()},
        "model_contract": "docs/problem_1_2_model_contract.md",
        "problem_1": {
            "exact_intersection": {
                "input": {"observations": exact_observations, "error_deg": 0.0, "target_radius_m": 1_800.0},
                "result": _localization_payload(exact_result),
                "expected_property": "two orthogonal exact rays intersect at one point",
            },
            "zero_degree_wrap": {
                "input": {"observations": wrapped_observations, "error_deg": 2.0, "target_radius_m": 1_800.0},
                "result": _localization_payload(wrapped_result),
                "expected_property": "the 359-degree cone is continuous across zero degrees",
            },
            "unbounded_single_bearing": {
                "input": {"observations": unbounded_observations, "error_deg": 1.0, "target_radius_m": None},
                "result": _localization_payload(unbounded_result),
                "expected_property": "one nonzero-error bearing has no finite diameter",
            },
            "equilateral_triangle": {
                "input_vertices_m": equilateral_vertices,
                "diameter_m": triangle_diameter_m,
                "diameter_circle_covers": triangle_covers,
                "diameter_circle_centers_m": triangle_centers,
                "expected_property": "a diameter circle does not necessarily cover a convex region",
            },
        },
        "problem_2": {
            "source_region_grid": {
                "first_point_m": first_point,
                "bearing_deg": 359.0,
                "error_deg": 1.0,
                "target_radius_m": 1_800.0,
                "maximum_receive_radius_m": 1_500.0,
                "grid_spacing_m": 100.0,
                "sample_count": len(direction_region_grid),
                "sample_points_m": direction_region_grid,
                "interpretation": "a finite approximation of U_1; reduce grid spacing before decision use",
            },
            "input": {
                "first_point_m": first_point,
                "source_samples_m": source_samples,
                "candidates_m": ((300.0, 0.0), (0.0, 100.0)),
                "guaranteed_receive_radius_m": 1_000.0,
            },
            "candidate_ranking": [_candidate_payload(item) for item in q2_ranked],
            "best_candidate": _candidate_payload(q2_ranked[0]),
            "strict_kernel": {
                "angle_tolerance_deg": 70.0,
                "guaranteed_receive_radius_m": 1_000.0,
                "points": [_candidate_payload(item) for item in q2_strict_kernel],
                "empty_means": "use the candidate_ranking as a soft rather than guaranteed recommendation",
            },
            "grid_refinement": q2_refinement,
            "expected_property": "the lateral candidate has a higher worst-case intersection-angle score than the collinear baseline",
            "limitation": "source_samples_m is a constructed finite proxy for U_1, not a simulator-derived source set.",
        },
        "reproduce": "python scripts/run_problem_1_2.py --output output/results/problem_1_2_validation.json",
    }


def _localization_payload(result: LocalizationResult) -> dict[str, Any]:
    """Convert a typed result to a stable JSON-compatible dictionary."""
    return {
        "status": result.status,
        "vertices_m": result.vertices,
        "diameter_m": result.diameter_m,
        "diameter_pairs_m": result.diameter_pairs,
        "diameter_circle_covers": result.diameter_circle_covers,
        "diameter_circle_centers_m": result.cover_centers,
    }


def _candidate_payload(candidate: Any) -> dict[str, Any]:
    """Convert one immutable candidate score to the report schema."""
    return {
        "point": list(candidate.point),
        "worst_case_sine": candidate.worst_case_sine,
        "max_distance_m": candidate.max_distance_m,
        "guaranteed_receivable": candidate.guaranteed_receivable,
    }


def _grid_refinement(first_point: tuple[float, float]) -> list[dict[str, Any]]:
    """Check whether a constructed Q2 ranking is stable as the grid is refined."""
    snapshots: list[dict[str, Any]] = []
    for grid_spacing_m in (200.0, 100.0, 50.0):
        source_points = sample_first_direction_region(
            first_point=first_point,
            bearing_deg=359.0,
            grid_spacing_m=grid_spacing_m,
            error_deg=1.0,
            target_radius_m=1_800.0,
            maximum_receive_radius_m=1_500.0,
        )
        candidates = sample_target_disk(grid_spacing_m=grid_spacing_m, target_radius_m=1_800.0)
        ranked = rank_second_points(
            first_point=first_point,
            source_points=source_points,
            candidates=candidates,
            guaranteed_receive_radius_m=1_000.0,
        )
        strict_kernel = strict_second_point_kernel(
            first_point=first_point,
            source_points=source_points,
            candidates=candidates,
            angle_tolerance_deg=60.0,
            guaranteed_receive_radius_m=1_000.0,
        )
        snapshots.append(
            {
                "grid_spacing_m": grid_spacing_m,
                "source_sample_count": len(source_points),
                "candidate_sample_count": len(candidates),
                "best_soft_candidate": _candidate_payload(ranked[0]),
                "strict_kernel_count": len(strict_kernel),
                "best_strict_candidate": _candidate_payload(strict_kernel[0]) if strict_kernel else None,
            }
        )
    return snapshots


def main() -> None:
    """Parse CLI arguments, generate evidence, and write it as UTF-8 JSON."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("output/results/problem_1_2_validation.json"),
        help="Destination JSON file (default: output/results/problem_1_2_validation.json).",
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    report = build_validation_report()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    LOGGER.info("wrote offline validation report to %s", args.output)


if __name__ == "__main__":
    main()
