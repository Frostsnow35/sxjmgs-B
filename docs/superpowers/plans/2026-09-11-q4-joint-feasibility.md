# Q4 Joint Feasibility Projection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a safe, semi-analytic location-orientation feasibility projector for Q4 that accelerates actions without weakening conservative clearance coverage.

**Architecture:** `joint_feasibility.py` owns the mathematical predicate and deterministic adaptive location sampling. `strategy.py` consumes its projected points only as optional action hints and retains the current polygon/MEC/coverage fallback. Tests run directly against the pure module and then use LocalEnv for a fixed-seed comparison.

**Tech Stack:** Python 3.12, standard library geometry helpers, existing `geometry.py`, existing LocalEnv; no new dependency and no simulator connection.

---

## File structure

- Create `issue/B/Q3&Q4 - exploration/joint_feasibility.py`: angular-arc feasibility predicate, convex-hull safe-negative detection, adaptive projector.
- Create `issue/B/Q3&Q4 - exploration/test_joint_feasibility.py`: `unittest` regression tests for the pure functions.
- Modify `issue/B/Q3&Q4 - exploration/strategy.py`: replace position-only sample use with optional joint-projection action hints; keep conservative fallback unchanged.
- Create `issue/B/Q3&Q4 - exploration/evaluate_joint_projection.py`: fixed-seed LocalEnv control/treatment evaluator.
- Modify `issue/B/Q3&Q4 - exploration/SOLUTION_NOTES.md` and `issue/B/context/draft_problem_4.md`: add the model, theorem boundary and evidence wording.

### Task 1: Define the pure joint predicate

**Files:**
- Create: `issue/B/Q3&Q4 - exploration/test_joint_feasibility.py`
- Create: `issue/B/Q3&Q4 - exploration/joint_feasibility.py`

- [x] **Step 1: Write failing tests for radius and angular feasibility**

```python
class JointFeasibilityTest(unittest.TestCase):
    def test_rejects_location_requiring_radius_above_1500(self):
        result = feasible_orientation((1600.0, 0.0), [((0.0, 0.0), 0.0)], [])
        self.assertFalse(result.feasible)

    def test_accepts_negative_explained_by_distance(self):
        result = feasible_orientation((100.0, 0.0), [((0.0, 0.0), 0.0)], [(1400.0, 0.0)])
        self.assertTrue(result.feasible)

    def test_rejects_conflicting_in_range_negative(self):
        result = feasible_orientation((100.0, 0.0), [((0.0, 0.0), 0.0)], [(50.0, 0.0)])
        self.assertFalse(result.feasible)
```

- [x] **Step 2: Run the tests to verify RED**

Run: `python -m unittest "test_joint_feasibility.py" -v` from `issue/B/Q3&Q4 - exploration`.

Expected: import failure because `joint_feasibility` does not exist.

- [x] **Step 3: Implement the smallest pure API**

```python
@dataclass(frozen=True)
class FeasibilityResult:
    feasible: bool
    r_min: float
    witness_angle_deg: float | None
    arc_count: int

def feasible_orientation(position, signals, no_signal_points) -> FeasibilityResult:
    """Test whether some source radius and directional half-plane explain observations."""
```

Represent every half-plane condition as an interval of width 180 degrees on a duplicated `[0, 720)` axis, intersect closed positive arcs and open negative arcs, and return one interval midpoint as a witness.

- [x] **Step 4: Run the tests to verify GREEN**

Run: `python -m unittest "test_joint_feasibility.py" -v`.

Expected: all Task 1 tests pass.

### Task 2: Add the convex-hull corollary and projector

**Files:**
- Modify: `issue/B/Q3&Q4 - exploration/test_joint_feasibility.py`
- Modify: `issue/B/Q3&Q4 - exploration/joint_feasibility.py`

- [x] **Step 1: Write failing tests for safe negatives and deterministic projection**

```python
def test_convex_hull_negative_requires_distance_above_1000(self):
    signals = [((-10.0, 0.0), 0.0), ((10.0, 0.0), 0.0), ((0.0, 10.0), 0.0)]
    self.assertFalse(feasible_orientation((0.0, 0.0), signals, [(0.0, 1.0)]).feasible)

def test_projector_keeps_a_known_feasible_location(self):
    points = project_positions(square_polygon(0.0, 200.0), [((0.0, 0.0), 0.0)], [(200.0, 200.0)], 50.0)
    self.assertIn((100.0, 0.0), points)
```

- [x] **Step 2: Run the new tests to verify RED**

Run: `python -m unittest "test_joint_feasibility.py" -v`.

Expected: failures because hull-safe exclusion and `project_positions` are absent.

- [x] **Step 3: Implement deterministic projection**

```python
def safe_negative_points(signals, no_signal_points) -> list[Point]:
    """Return only negatives inside the convex hull of positive signal points."""

def project_positions(poly, signals, no_signal_points, max_cell_m=25.0) -> list[ProjectedPoint]:
    """Sample polygon cells deterministically and retain feasible position-orientation witnesses."""
```

Cells wholly outside a safe-negative disk may be skipped; boundary cells remain and are sampled, so projection cannot be confused with a proof-level inner or outer approximation.

- [x] **Step 4: Run the tests to verify GREEN**

Run: `python -m unittest "test_joint_feasibility.py" -v`.

Expected: all Task 1 and Task 2 tests pass.

### Task 3: Integrate hints without weakening fallback

**Files:**
- Modify: `issue/B/Q3&Q4 - exploration/test_joint_feasibility.py`
- Modify: `issue/B/Q3&Q4 - exploration/strategy.py`

- [x] **Step 1: Write a failing LocalEnv-free integration test**

```python
def test_empty_joint_projection_calls_conservative_cover(monkeypatch):
    monkeypatch.setattr(strategy.joint, "project_positions", lambda *args, **kwargs: [])
    monkeypatch.setattr(strategy, "_cover_and_clear", sentinel_cover)
    strategy.clear_channel_q4(FakeInterface(), 1, two_signal_channel_data())
    self.assertTrue(sentinel_cover.called)
```

- [x] **Step 2: Run the test to verify RED**

Run: `python -m unittest "test_joint_feasibility.py" -v`.

Expected: failure because `strategy.joint` does not exist and Q4 has no joint hint branch.

- [x] **Step 3: Implement only optional hint consumption**

Call `project_positions` after conservative `poly` construction. Keep the MEC as the sole remeasurement point; use the projected MEC center only to prioritize the first existing point of a subsequent coverage route. Retain every coverage candidate and all existing MEC/`_cover_and_clear` branches unchanged.

- [x] **Step 4: Run the tests to verify GREEN**

Run: `python -m unittest "test_joint_feasibility.py" -v`.

Expected: all tests pass, including fallback behavior.

### Task 4: Produce reproducible local evidence and update explanations

**Files:**
- Create: `issue/B/Q3&Q4 - exploration/evaluate_joint_projection.py`
- Modify: `issue/B/Q3&Q4 - exploration/SOLUTION_NOTES.md`
- Modify: `issue/B/context/draft_problem_4.md`

- [x] **Step 1: Implement paired fixed-seed evaluator**

Run seeds 2000–2049 once with the joint hint disabled and once enabled. Save only structured summary fields: seed, source count, cleared count, virtual time, measure count, clear count, projection-call count and projection-hit count.

- [x] **Step 2: Run the evaluator and inspect structured output**

Run: `python evaluate_joint_projection.py --cases 50 --seed 2000 --output joint_projection_localenv.json`.

Expected: every row reports the same source count and clearance count across paired modes; any mismatch is reported rather than suppressed.

- [x] **Step 3: Update model text from generated numbers only**

State the latent-variable equations, the $r_{\min}$ reduction, the convex-hull corollary, and that the projection is an acceleration heuristic while conservative coverage retains the finite-clearance guarantee. Mark all evaluator figures as LocalEnv evidence, not simulator or formal-test evidence.

- [x] **Step 4: Run final lightweight verification**

Run: `python -m unittest "test_joint_feasibility.py" -v; python -m compileall joint_feasibility.py strategy.py evaluate_joint_projection.py`.

Expected: unit tests pass and all three modules compile without error.
