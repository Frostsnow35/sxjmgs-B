# 问题三安全演练重构 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 构建可复核、仅在用户确认演练后调用接口的问题三搜索、定位与清除入口，并以解析覆盖保证替代队友的固定网格扫描。

**Architecture:** `protocol.py` 只负责串行、幂等和日志；`strategy.py` 只负责扫描点、复测点和清除覆盖候选；运行脚本把协议响应转为观测并调用既有问题一几何模块。任何模拟器动作均由显式演练参数和用户确认的接口就绪状态共同约束。

**Tech Stack:** Python 3.12、标准库、`requests`、pytest、既有 `src.localization_geometry`。

---

## 文件结构

| 路径 | 职责 |
|---|---|
| `src/problem_3/protocol.py` | 请求 ID、HTTP/业务响应校验、JSONL、现实 deadline。 |
| `src/problem_3/strategy.py` | 7 点覆盖、保证复测、待处理频道和 20 m 覆盖候选。 |
| `scripts/run_problem_3_rehearsal.py` | 显式演练入口和结构化汇总。 |
| `tests/test_problem_3_protocol.py` | 协议层的假会话测试。 |
| `tests/test_problem_3_strategy.py` | 几何保证与清除候选覆盖测试。 |
| `docs/problem_3_model_contract.md` | 模型、数据流、参数、限制和演练口径。 |
| `.gitignore` | 忽略本地运行时、原始日志和 Python 缓存。 |

### Task 1: 安全协议层

**Files:**
- Create: `src/problem_3/__init__.py`
- Create: `src/problem_3/protocol.py`
- Create: `tests/test_problem_3_protocol.py`

- [ ] **Step 1: 写失败测试，表达拒绝非演练入口和 HTTP 双校验**

```python
from pathlib import Path
import pytest
from src.problem_3.protocol import RehearsalClient, SimulatorProtocolError

def test_client_refuses_requests_without_rehearsal_confirmation(tmp_path: Path) -> None:
    client = RehearsalClient("http://127.0.0.1:2026", "team", tmp_path / "x.jsonl", rehearsal_confirmed=False)
    with pytest.raises(SimulatorProtocolError, match="rehearsal"):
        client.enter()

def test_client_rejects_http_error_even_when_body_claims_accepted(tmp_path: Path) -> None:
    client = RehearsalClient.from_fake_response(500, {"accepted": True}, tmp_path / "x.jsonl")
    with pytest.raises(SimulatorProtocolError, match="HTTP 500"):
        client.enter()
```

- [ ] **Step 2: 运行失败测试**

Run: `python -m pytest -p no:cacheprovider tests/test_problem_3_protocol.py -q`  
Expected: `ModuleNotFoundError: No module named 'src.problem_3'`.

- [ ] **Step 3: 实现最小协议客户端**

```python
class SimulatorProtocolError(RuntimeError):
    """Raised when a simulator action cannot be trusted as executed."""

class RehearsalClient:
    def __init__(self, base_url: str, robot_id: str, log_path: Path, rehearsal_confirmed: bool, session: requests.Session | None = None) -> None: ...
    def enter(self) -> dict[str, Any]: ...
    def measure(self, position: Point, channel: int) -> dict[str, Any]: ...
    def clear(self, position: Point, channel: int) -> dict[str, Any]: ...
    def exit(self) -> dict[str, Any]: ...
```

`_post` 必须生成或接收固定 `request_id`、对同一 payload 重试最多三次、先检查 `status_code` 再检查 `accepted is True`，并将时间、路径、payload、HTTP 状态、响应或异常逐行写入 UTF-8 JSONL。

- [ ] **Step 4: 运行协议测试**

Run: `python -m pytest -p no:cacheprovider tests/test_problem_3_protocol.py -q`  
Expected: all tests pass.

- [ ] **Step 5: 提交协议层**

```bash
git add src/problem_3 tests/test_problem_3_protocol.py
git commit -m "feat: add safe rehearsal protocol client"
```

### Task 2: 覆盖与复测策略

**Files:**
- Create: `src/problem_3/strategy.py`
- Create: `tests/test_problem_3_strategy.py`

- [ ] **Step 1: 写失败测试，固定 7 点覆盖界和复测保证**

```python
import math
from src.problem_3.strategy import initial_scan_points, guaranteed_second_point, worst_initial_scan_distance

def test_seven_scan_points_cover_target_disk_at_minimum_receive_radius() -> None:
    assert worst_initial_scan_distance() < 1000.0
    assert len(initial_scan_points()) == 7

def test_sideways_second_point_is_receivable_under_bearing_error_bounds() -> None:
    first = (0.0, 0.0)
    second = guaranteed_second_point(first, 0.0)
    assert second == (750.0, 600.0)
    assert math.dist(second, (1500.0 * math.cos(math.radians(-1)), 1500.0 * math.sin(math.radians(-1)))) < 1000.0
```

- [ ] **Step 2: 运行失败测试**

Run: `python -m pytest -p no:cacheprovider tests/test_problem_3_strategy.py -q`  
Expected: import failure because `strategy.py` is absent.

- [ ] **Step 3: 实现纯函数策略模块**

```python
def initial_scan_points(radius_m: float = 1200.0) -> tuple[Point, ...]: ...
def worst_initial_scan_distance(target_radius_m: float = 1800.0, scan_radius_m: float = 1200.0) -> float: ...
def guaranteed_second_point(first: Point, bearing_deg: float, lateral_sign: float = 1.0) -> Point: ...
def clearance_grid(bounds: tuple[float, float, float, float], radius_m: float = 20.0) -> tuple[Point, ...]: ...
```

`clearance_grid` 的间距取 `radius_m * sqrt(2)`，并向四周补一个半间距，使所给轴对齐框内任一点到某候选点的距离不超过 `radius_m`。

- [ ] **Step 4: 运行策略测试**

Run: `python -m pytest -p no:cacheprovider tests/test_problem_3_strategy.py -q`  
Expected: all tests pass.

- [ ] **Step 5: 提交策略层**

```bash
git add src/problem_3/strategy.py tests/test_problem_3_strategy.py
git commit -m "feat: add guaranteed problem three scan geometry"
```

### Task 3: 编排、定位接口和可追溯汇总

**Files:**
- Create: `scripts/run_problem_3_rehearsal.py`
- Create: `tests/test_problem_3_runner.py`
- Create: `docs/problem_3_model_contract.md`

- [ ] **Step 1: 写失败测试，约束单次观测不被跳过**

```python
from src.problem_3.strategy import pending_channels_after_scan

def test_channel_with_one_direction_is_pending_for_guaranteed_remeasurement() -> None:
    observations = {7: [((0.0, 0.0), 20.0)]}
    assert pending_channels_after_scan(observations, cleared_channels=set()) == (7,)
```

- [ ] **Step 2: 运行失败测试**

Run: `python -m pytest -p no:cacheprovider tests/test_problem_3_runner.py -q`  
Expected: import failure for `pending_channels_after_scan`.

- [ ] **Step 3: 实现编排入口和模型合同**

运行脚本必须：

```text
--robot-id <id> --rehearsal-confirmed --output <summary.json> --log <actions.jsonl>
```

处理顺序必须是：7 点按固定环形路线扫描全部频道 → `near` 立即清除 → 每个一次 `direction` 的频道调用 `guaranteed_second_point` → 以全部 `direction` 调用 `locate_from_bearings` → 先对定位摘要给出的中心候选清除 → 失败时对保守边界框调用 `clearance_grid`，直到 `success` 或明确记录未完成。

模型合同必须记录：扫描半径 1200 m 来自式（4）的覆盖证明；1500/1000 m、1°、20 m、5 m 均来自题面或接口；当前回退网格是保证性手段，需以演练统计其调用频率和耗时。

- [ ] **Step 4: 运行整套离线测试**

Run: `python -m pytest -p no:cacheprovider tests -q`  
Expected: existing 14 tests plus problem-three tests all pass.

- [ ] **Step 5: 提交编排与合同**

```bash
git add scripts/run_problem_3_rehearsal.py tests/test_problem_3_runner.py docs/problem_3_model_contract.md src/problem_3/strategy.py
git commit -m "feat: add auditable question three rehearsal runner"
```

### Task 4: 工作区迁移和演练验证

**Files:**
- Modify: `.gitignore`
- Delete: `q3-solution/code/__pycache__/robot_client.cpython-313.pyc`
- Modify: `README.md`, `state/decision_log.json`, `task_plan.md`, `findings.md`, `progress.md`
- Create: `output/results/problem_3/<utc>_summary.json` after a user-confirmed rehearsal

- [ ] **Step 1: 删除已跟踪缓存并更新协作文档**

执行 `git rm --cached q3-solution/code/__pycache__/robot_client.cpython-313.pyc`；保留 `q3-solution/` 文档和脚本原件。README 增加“问题三新入口只用于用户确认的演练”与结果目录说明。

- [ ] **Step 2: 演练前检查**

Run: `python scripts/run_problem_3_rehearsal.py --help`  
Expected: required `--robot-id` 和 `--rehearsal-confirmed` 在帮助中可见；不得有任何 HTTP 请求。

- [ ] **Step 3: 仅在用户确认接口属于问题三演练且已就绪后运行**

Run: `python scripts/run_problem_3_rehearsal.py --robot-id <current-login-id> --rehearsal-confirmed --output output/results/problem_3/<utc>_summary.json --log logs/problem_3/<utc>.jsonl`  
Expected: 每个动作有 JSONL 记录；汇总区分 `cleared_channels`、`unresolved_channels` 与 `reported_total_sources`（若用户从界面提供）。

- [ ] **Step 4: 核对与提交**

Run: `python -m json.tool output/results/problem_3/<utc>_summary.json > $null; python -m pytest -p no:cacheprovider tests -q; git diff --check`  
Expected: JSON 可解析、全部测试通过、无空白错误。

- [ ] **Step 5: 提交演练版本**

```bash
git add .gitignore README.md state/decision_log.json task_plan.md findings.md progress.md output/results/problem_3
git rm --cached q3-solution/code/__pycache__/robot_client.cpython-313.pyc
git commit -m "chore: organize question three rehearsal artifacts"
```

## Self-review

- 覆盖：协议、首次覆盖、二次保证、单次观测、清除兜底、日志、门禁、迁移和演练结果均有任务。
- 占位符：没有用于实现步骤的不定指令；演练输出文件名以实际 UTC 运行时生成，不能预填。
- 类型：`Point` 为 `tuple[float, float]`；`RehearsalClient` 只在协议层依赖 `requests`；策略模块保持无网络副作用。
