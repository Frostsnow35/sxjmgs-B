# 问题一、二几何定位与选点求解 Implementation Plan

> **For agentic workers:** 按 TDD 顺序执行；每一项先验证失败、再写最小实现、再验证通过。

**Goal:** 交付一个不依赖模拟器的可复现几何求解模块，完成问题一定位区域/直径/覆盖判定，并给出问题二第二检测点的鲁棒候选排序与验证结果。

**Architecture:** `src/localization_geometry.py` 只承担二维几何与候选点评分，不读取题面或调用网络。`scripts/run_problem_1_2.py` 仅组合已验证的公共函数并写入结构化结果。`tests/` 用解析构造例约束边界、退化和鲁棒评分行为。

**Tech Stack:** Python 3.12、标准库、pytest；不新增第三方依赖，不启动模拟器。

---

### Task 1: 更新求解状态与模型合同

**Files:**
- Modify: `E:\CUMCM2026\task_plan.md`
- Modify: `E:\CUMCM2026\todo.md`
- Create: `E:\CUMCM2026\docs\problem_1_2_model_contract.md`

- [x] 将用户的“开始问题一和问题二的求解”记录为已批准的离线求解授权；保留问题三、四模拟器门禁。
- [x] 固定坐标、角度、误差闭角域、空/退化/无界情形及问题二的鲁棒目标，且明确无真实测向数据时只输出验证案例，不伪造赛题数值。

### Task 2: 为问题一写红灯测试

**Files:**
- Create: `E:\CUMCM2026\tests\test_localization_geometry.py`

- [x] **Step 1: 写失败测试**

```python
def test_intersection_of_two_exact_bearings_is_a_point() -> None:
    result = locate_from_bearings(((0.0, 0.0, 0.0), (1.0, 0.0, 90.0)), error_deg=0.0)
    assert result.status == "point"
    assert result.diameter_m == 0.0
```

- [x] **Step 2: 验证失败**

Run: `python -m pytest tests/test_localization_geometry.py -q`

Expected: 因 `src.localization_geometry` 尚不存在而失败。

### Task 3: 实现问题一最小几何核心

**Files:**
- Create: `E:\CUMCM2026\src\localization_geometry.py`
- Test: `E:\CUMCM2026\tests\test_localization_geometry.py`

- [x] **Step 1: 实现 API**

```python
def locate_from_bearings(
    observations: Sequence[Observation], error_deg: float = 1.0
) -> LocalizationResult:
    """Return the closed bearing-region intersection and its geometry summary."""
```

- [x] **Step 2: 覆盖枚举边界直线交点、可行性、凸包、有界性、直径和直径圆判定。**
- [x] **Step 3: 验证通过**

Run: `python -m pytest tests/test_localization_geometry.py -q`

Expected: 所有问题一测试通过。

### Task 4: 为问题二写红灯测试并实现评分

**Files:**
- Modify: `E:\CUMCM2026\tests\test_localization_geometry.py`
- Modify: `E:\CUMCM2026\src\localization_geometry.py`

- [x] **Step 1: 添加失败测试**

```python
def test_perpendicular_second_point_has_better_worst_case_geometry() -> None:
    source_points = ((100.0, 0.0), (200.0, 0.0))
    assert worst_case_intersection_sine((0.0, 100.0), (0.0, 0.0), source_points) > \
        worst_case_intersection_sine((300.0, 0.0), (0.0, 0.0), source_points)
```

- [x] **Step 2: 验证失败后，最小实现 `intersection_sine`、`worst_case_intersection_sine` 与网格候选排序。**
- [x] **Step 3: 验证通过**

Run: `python -m pytest tests/test_localization_geometry.py -q`

Expected: 所有问题一、二测试通过。

### Task 5: 运行验证实验并冻结证据

**Files:**
- Create: `E:\CUMCM2026\scripts\run_problem_1_2.py`
- Create: `E:\CUMCM2026\output\results\problem_1_2_validation.json`
- Modify: `E:\CUMCM2026\findings.md`
- Modify: `E:\CUMCM2026\progress.md`
- Modify: `E:\CUMCM2026\state\decision_log.json`

- [x] **Step 1: 运行脚本**

Run: `python scripts/run_problem_1_2.py --output output/results/problem_1_2_validation.json`

Expected: 生成含输入、代码口径、问题一解析例和问题二基线对照的 JSON。

- [x] **Step 2: 验证证据**

Run: `python -m pytest -p no:cacheprovider tests -q; python -m json.tool output/results/problem_1_2_validation.json > $null; git diff --check`

Expected: 测试、JSON 解析和空白符检查全部通过。

- [ ] **Step 3: 提交协作分支成果**

```powershell
git add docs src scripts tests output/results task_plan.md todo.md findings.md progress.md state/decision_log.json
git commit -m "feat: solve geometry baselines for problems 1 and 2"
git push origin main
```
