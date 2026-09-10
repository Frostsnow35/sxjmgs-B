"""Tests for the reproducible offline validation-report entry point."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from scripts.run_problem_1_2 import build_validation_report


def test_validation_report_records_offline_cases_without_simulator_claims() -> None:
    """The frozen report must identify its scope and preserve core evidence."""
    report = build_validation_report()

    assert report["schema_version"] == "1.0"
    assert report["scope"] == "offline_constructed_validation_cases_only"
    assert report["simulator_called"] is False
    assert report["problem_1"]["exact_intersection"]["result"]["status"] == "point"
    assert report["problem_1"]["equilateral_triangle"]["diameter_circle_covers"] is False
    assert report["problem_2"]["best_candidate"]["point"] == [0.0, 100.0]
    assert report["problem_2"]["source_region_grid"]["grid_spacing_m"] == 100.0
    assert report["problem_2"]["source_region_grid"]["sample_count"] > 0
    assert report["problem_2"]["strict_kernel"]["points"][0]["point"] == [0.0, 100.0]
    refinements = report["problem_2"]["grid_refinement"]
    assert [entry["grid_spacing_m"] for entry in refinements] == [200.0, 100.0, 50.0]
    assert refinements[2]["source_sample_count"] > refinements[0]["source_sample_count"]
    assert all(entry["candidate_sample_count"] > 0 for entry in refinements)


def test_runner_executes_directly_from_repository_root() -> None:
    """The documented command must work without manually setting PYTHONPATH."""
    output_path = Path("tmp/direct_runner_validation.json")
    completed = subprocess.run(
        [
            sys.executable,
            "scripts/run_problem_1_2.py",
            "--output",
            str(output_path),
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert output_path.is_file()
