# 问题三安全演练与保证型搜索设计

> 状态：已确认设计，待按实现计划编码  
> 日期：2026-09-10  
> 影响范围：问题三演练客户端、搜索策略、测试、结果与日志目录；不改变问题一、二几何模块的既有语义。

## Why

队友的 9 点/17 点脚本存在覆盖不足、文档与入口不一致、把确定误差按最小二乘处理、失败后放弃频道、HTTP 状态未校验、日志没有持久化等问题。它不能支持题目“确保所有干扰源被清除”的要求，也不足以形成可复核的演练证据。

本设计把“首次发现”“保证复测”“带误差定位”“成功清除”拆成不同的可检验环节。演练只能验证实现与策略表现；不把单局演练数据写成正式测试结果。

## What Changes / BREAKING

- 新增 `src/problem_3/`，将协议安全、几何策略和清除候选生成从队友的单文件脚本中拆开。
- 新增 `scripts/run_problem_3_rehearsal.py`。该入口只接受显式 `--rehearsal-confirmed`，并把每次请求及响应写入本地 JSONL；它不点击 UI，也不能自行判断模拟器是否为正式测试，仍依赖用户先确认演练模式。
- 新增 `tests/test_problem_3_*.py`，先验证覆盖界、保证复测、幂等重试、HTTP/`accepted` 双校验、清除候选覆盖和日志格式。
- 新增 `docs/problem_3_model_contract.md`，记录模型输入、输出、保证、局限、演练口径及与问题一、二的接口。
- 队友原件 `q3-solution/` 保持不动，迁移后的新入口不再调用其中的 `auto_search.py` 或 `test_run.py`。
- 已跟踪的 `q3-solution/code/__pycache__/robot_client.cpython-313.pyc` 将从 Git 索引删除；`.gitignore` 继续负责忽略本地缓存和原始演练 CSV。

## Impact

| 对象 | 影响 |
|---|---|
| 问题一 | `locate_from_bearings` 接收某频道的多条 `(x,y,svd_deg)`，生成闭角域定位摘要；不改变其误差界或圆域口径。 |
| 问题二 | 首次 `direction` 的保证型第二点使用同一“接收距离下界 + 交会角”思想，但使用可解析的侧向构造作为运行时基线。 |
| 问题三 | 所有已发现频道都进入待处理队列；只有 `clear_result="success"` 才视作已清除。 |
| 演练证据 | 原始请求—响应 JSONL 写入 `logs/problem_3/`（忽略）；脱敏汇总 JSON/CSV 写入 `output/results/problem_3/`（可提交）。 |
| 正式测试 | 本设计不授权、不开启或模拟正式测试。任何正式测试仍需用户当次明确批准，并使用单独的冻结版本。 |

## Architecture

```text
run_problem_3_rehearsal.py
  ├─ RehearsalClient          串行 HTTP、幂等重试、deadline、JSONL
  ├─ SevenPointScanner        首次覆盖与按频道收集 direction/near
  ├─ SecondaryPointPlanner    单条 direction 的保证型侧向复测
  ├─ locate_from_bearings     复用问题一闭角域结果
  └─ ClearancePlanner         先试定位中心；失败时生成 ≤20 m 覆盖网格
```

协议层不知道模拟器 UI 的“演练/正式”状态。`--rehearsal-confirmed` 只是本地防误触护栏，不能替代用户在 UI 中确认演练；在未确认界面与接口就绪时，入口不得执行。

## Mathematical Guarantees

### 首次扫描

首次点集取圆心与半径 $1200\ \mathrm{m}$ 的正六边形顶点。对任意源 $G\in\Omega$，到最近扫描点的距离最多为

$$
\sqrt{1800^2+1200^2-2\times1800\times1200\cos30^\circ}
=968.9016\ \mathrm{m}<1000\ \mathrm{m}.
$$

所以全向源在最小有效接收半径下至少有一次 `direction` 或 `near`。`near` 当场 `/clear`；`direction` 保留为该频道的第一条观测。

### 保证型第二检测点

给定首测点 $S$ 和示向度 $\theta$，设 $u(\theta)$ 为其单位方向，$v(\theta)$ 为逆时针法向，取

$$S'=S+750u(\theta)+600v(\theta).$$

在 $r\in(5,1500]$、方位误差 $\eta\in[-1^\circ,1^\circ]$ 下，$\|G-S'\|_2\le976.861\ \mathrm{m}<1000\ \mathrm{m}$，第二次接收有保证；同时 $|\sin\phi|>0.600$，避免近共线交会。该点可能在目标圆外，但接口明确允许提交圆外坐标。

### 清除闭环

两次及以上观测进入问题一的闭角域交集，不能以最小二乘点估计代替。先在定位区域中心的候选点 `/clear`。若未成功，以边长 $20\sqrt2\ \mathrm{m}$ 的方格覆盖一个包含定位区域的有界框；每个可行源点距某格点不超过 $20\ \mathrm{m}$，逐点清除构成有限兜底。该兜底的开销需要在演练中报告，不能被描述为高效策略。

## SHALL Requirements and Acceptance Scenarios

1. WHEN 目标圆内任意全向源的有效接收半径取最小值 $1000\ \mathrm{m}$，THEN 首次扫描点集 SHALL 保证至少一个点可收到该源。
2. WHEN 某频道仅收到一条 `direction`，THEN 策略 SHALL 安排保证型侧向复测，而不是跳过该频道。
3. WHEN `/measure` 或 `/clear` 返回 HTTP 非 2xx、非 JSON 或 `accepted=false`，THEN 客户端 SHALL 记录状态并停止当前策略，不将动作计为已执行。
4. WHEN 网络中断发生在已发出请求后，THEN 重试 SHALL 复用完全相同的 payload 与 `request_id`；新动作 SHALL 使用新 ID。
5. WHEN `/clear` 返回非 `success`，THEN 频道 SHALL 保留在待处理集合，直到定位更新或兜底候选耗尽。
6. WHEN 演练结束，THEN 程序 SHALL 生成原始 JSONL 路径、汇总指标、已清除频道、未完成频道、执行命令及配置哈希；汇总不得把“发现频道”误标为“干扰源总数”。
7. WHEN 未传入 `--rehearsal-confirmed`，THEN 入口 SHALL 在任何 HTTP 请求前退出。

## Removed / Migration

- 移除：`q3-solution/code/__pycache__/robot_client.cpython-313.pyc` 的版本跟踪。原因：缓存不可复现且 `.gitignore` 已声明忽略。迁移：无。
- 不废弃：`q3-solution/` 中的队友 DOCX 与脚本。原因：保留基线复核来源。迁移：新代码不再依赖它们；已冻结的基线摘要继续保留在 `output/results/problem_3/`。
