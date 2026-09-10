# 问题三模型合同：仅限模拟演练的可审计清除编排

> 更新日期：2026-09-11
>
> 状态：**已完成：入口实现与离线 FakeClient 测试；需要验证：后续人工确认后的实际模拟演练。**
> 影响范围：`scripts/run_problem_3_rehearsal.py`、`src/problem_3/strategy.py` 的待处理频道纯函数、问题三演练摘要；不修改问题一闭角域或协议层语义。

## Why

问题三需要在题设圆域内发现并清除全向干扰源。已有基线脚本不能把“一条 `direction`”频道纳入后续处理，也不能把带确定性误差界的示向度直接当成最小二乘（least squares, LS）点估计；这会遗漏源或过度缩小其可行区域。

本合同将发现、复测、定位、清除和审计拆开：只在操作者已确认问题三**模拟演练**时运行；每个成功或失败状态都有可复核记录；不把发现频道数称为总源数，也不伪造任何实际演练结果。

## What Changes / BREAKING

- 已完成：新增 `scripts/run_problem_3_rehearsal.py`，提供注入式 `run_rehearsal(...)` 与命令行入口。它按固定七点、频道 1--20 进行扫描，处理 `near`、单条 `direction` 复测、问题一闭角域定位与有限网格回退。
- 已完成：新增 `pending_channels_after_scan(observations, cleared_channels)`；所有至少一条 `direction` 且未清除的频道都会返回，包括仅一条观测的频道。
- 已完成：新增离线 FakeClient 测试，验证演练门禁、动作顺序、时限、回退范围与摘要口径；测试不访问 `127.0.0.1` 或任何真实网络。
- BREAKING：无对外协议变更。旧 `q3-solution/` 基线脚本不是本入口的依赖，不能再作为本合同所述保证流程的执行证据。

## Impact

| 对象 | 影响 |
|---|---|
| 问题一 | 每个频道的全部 `direction` 被转成 `(x, y, bearing_deg)`，调用 `locate_from_bearings(error_deg=1, target_radius_m=1800)`；运行器不以 LS 替代该闭角域交集。 |
| 问题二 | 单条首测方向以 `S'=S+750u(θ)+600v(θ)` 调用 `guaranteed_second_point`，随后才定位。 |
| 问题三 | 只有 `/clear` 的 `clear_result == "success"` 才进入 `cleared_channels`；`no_signal`、清除失败和候选耗尽均保留未完成频道。 |
| 协议与日志 | `RehearsalClient` 负责 JSONL 请求--响应日志；Runner 只引用日志路径并写脱敏汇总，不记录 robot ID、接口密钥或 UI 测试码。 |
| 正式测试 | 本入口不判断 UI 模式、不开启也不模拟正式测试。正式测试必须在当次另获用户明确授权并冻结版本。 |

## 输入、输出与运行边界

### 命令行接口

| 参数 | 类型 / 默认值 | 说明 |
|---|---|---|
| `--robot-id` | `str`，必填 | 仅传给协议客户端；不会写入摘要。 |
| `--rehearsal-confirmed` | `bool`，默认 `false` | 必须显式出现。缺失时入口在创建客户端前退出，动作数为 0。 |
| `--output` | `Path`，必填 | UTF-8 汇总 JSON；父目录自动创建。 |
| `--log` | `Path`，必填 | `RehearsalClient` 的 JSONL 审计日志路径。 |
| `--base-url` | `str`，默认 `http://127.0.0.1:2026` | 已确认演练模拟器的基础地址；只在确认标志存在后传入客户端。 |

入口提示为“仅问题三模拟演练；不判断 UI 模式”。`--help` 只解析参数，不构造客户端或发送请求。

### `run_rehearsal` 内部接口

`run_rehearsal(client, output_path, action_log_path, monotonic=..., command_summary=...) -> dict[str, Any]` 接受有 `enter/measure/clear/exit` 四个同步方法的客户端；测试通过显式 FakeClient 和单调时钟注入执行。该函数始终尽力生成摘要，响应格式或 `SimulatorProtocolError` 出错后停止新的业务动作。

摘要 JSON 至少包含 `schema/version`、`run_mode="rehearsal"`、`status`、`stop_reason`、已清除/未完成/已发现频道、`reported_total_sources: null`、最后已接受响应的 `virtual_time_s`、enter 响应的 `remaining_real_duration_s`、日志路径、实际与配置扫描点数、每频道观测数、定位状态、脱敏命令参数和配置哈希。

`reported_total_sources` 恒为 `null`，等待操作者从 UI 获取并另行报告；频道数不是源总数。

## 模型与编排口径

### 七点固定扫描

扫描点为圆心和半径 $1200\ \mathrm{m}$ 的正六边形六顶点，顺序固定为圆心后逆时针顶点。目标圆半径为 $1800\ \mathrm{m}$ 时，任一点到最近扫描点的严格上界为

$$
\sqrt{1800^2+1200^2-2\times1800\times1200\cos30^\circ}
=968.9016\ \mathrm{m}<1000\ \mathrm{m}.
$$

因此在题设最小有效接收半径 $1000\ \mathrm{m}$ 下，每个全向源至少会得到一次 `direction` 或 `near`。每个扫描点都按频道 1--20 顺序测量；`near` 在原位置立即 `/clear`，仅成功才标记清除，`no_signal` 只记录为无信号。

### 750+600 保证复测与闭角域定位

对每个仍仅有一条 `direction` 的频道，若首测点为 $S$、读数为 $\theta$，取

$$S'=S+750u(\theta)+600v(\theta),$$

其中 $u$ 是示向单位向量、$v$ 是其逆时针法向。根据当前题设的接收 range（最小保证半径 $1000\ \mathrm{m}$、首测可能距离至 $1500\ \mathrm{m}$）和 $\pm1^\circ$ 误差口径，该构造为第二次接收和非退化交会角提供保证；它不是沿首条方位线的共线移动。二测 `near` 同样立即清除；二测 `no_signal` 或清除失败不会删除该频道。

随后对每个未清除且至少一条 `direction` 的频道，调用问题一的 `locate_from_bearings`。该函数计算带 $\pm1^\circ$ 误差和半径 $1800\ \mathrm{m}$ 目标圆的**闭角域交集**；不得改为 LS，也不得将不确定边界压成单一估计点。

### 清除回退、分辨率与时限

先按 `LocalizationResult.cover_centers` 逐点 `/clear`。若仍未成功，`clearance_grid` 用 $20\ \mathrm{m}$ 清除半径和轴向最大步长 $20\sqrt2\ \mathrm{m}$ 生成候选：每个方格点到最近候选不超过 $20\ \mathrm{m}$。

- 对 `point`、`segment`、`polygon` 且有顶点的定位结果，回退框是其顶点轴对齐 bounding box。
- 对 `empty`、`unbounded`、`disk_clipped`，或无顶点的任何状态，回退框必须是完整目标方框 $(-1800,-1800,1800,1800)$，不能因圆弧或不充分顶点而低估可行区域。
- `clearance_grid` 有 `100000` 候选保护；默认完整目标方框为 $129\times129=16641$ 个基础候选，低于上限。

完整全局网格是**保证性回退，不是高效性结论**：在真实时限内可能耗时很长，候选耗尽或时限到达时频道应留在 `unresolved_channels`，不得声称已清除。

真实时限只认 `/enter` 成功响应的 `remaining_real_duration_s`；不会假设固定 1200 秒。通过可注入单调时钟建立 deadline，每项新的 `measure` 或 `clear` 前预留至少 5 秒。字段缺失、非数或余量不足时停止新动作并写明原因；若已经进入会话，仍尽力只调用一次 `/exit`，其失败不能覆盖先前主失败原因。

## SHALL 要求与 WHEN/THEN 验收场景

1. WHEN 首次扫描返回 `{7: [((0, 0), 20)]}` 且清除集为空，THEN `pending_channels_after_scan` SHALL 返回 `(7,)`。
2. WHEN 已传入 `--rehearsal-confirmed` 且 `/enter` 成功，THEN Runner SHALL 按七个固定点、每点频道 1--20 的完整顺序扫描；`near` SHALL 紧跟同点同频道的 `/clear`。
3. WHEN 某未清除频道在扫描结束仅有一条 `direction`，THEN Runner SHALL 先调用 `guaranteed_second_point` 并在返回点测量同一频道；`no_signal` 或清除失败 THEN SHALL 仍将其置于未完成处理路径。
4. WHEN 频道有一条或多条 `direction` 且未清除，THEN Runner SHALL 以 `(x,y,bearing_deg)` 调用 `locate_from_bearings(error_deg=1, target_radius_m=1800)`；不得调用 LS 替代物。
5. WHEN 定位中心清除未成功，THEN Runner SHALL 依状态选择保守 bbox 并逐点执行有限 `clearance_grid`，直到成功、候选耗尽或时限停止。
6. WHEN `/enter` 的 `remaining_real_duration_s` 缺失、非数或小于安全余量，THEN Runner SHALL 不再开始 `measure`，摘要 SHALL 写出停止原因；不得使用固定时限猜测。
7. WHEN 协议抛出 `SimulatorProtocolError` 或响应缺少约定字段，THEN Runner SHALL 停止后续动作，写失败摘要；已成功 enter 时 SHALL 尽力 `/exit` 一次，而首次 enter 失败时 SHALL 不调用 `/exit`。
8. WHEN 未传 `--rehearsal-confirmed` 或调用 `--help`，THEN Runner SHALL 不构建真实客户端、不发送 HTTP 请求；未确认摘要 SHALL 不含 robot ID。
9. WHEN 汇总写入，THEN 它 SHALL 为 UTF-8 JSON、创建父目录、将发现频道与 `reported_total_sources=null` 分开，并提供可复算的配置哈希。

## 验证状态与局限

- 已完成：离线 FakeClient 测试验证七点扫描、立即清除、单方向复测、闭角域参数、失败频道保留、完整方框回退、真实时限门禁、帮助页和摘要脱敏。
- 已完成：策略网格已有离线几何测试，验证 $20\ \mathrm{m}$ 覆盖条件和 `100000` 候选保护。
- 需要验证：实际模拟演练只能在后续人工确认 UI 状态与授权后执行；本次未发送 HTTP、没有任何演练数值结果可报告。
- 局限：全局回退网格可能消耗大量真实时间；它不能被表述为高效搜索。若频繁触发，应在保持保证口径的前提下改进复测或增加经验证观测，且同步更新本合同与技术文档。

## REMOVED / Migration

- REMOVED：无已删除接口或数据。
- Migration：从旧 `q3-solution/` 基线脚本迁移时，改由本入口生成 JSONL 路径与脱敏摘要；旧脚本输出不可与本合同的 `status`、`unresolved_channels` 或配置哈希混用。任何改变误差界、扫描半径、复测构造、清除网格或时间余量的修改，都必须同步更新本合同、离线测试和技术文档。
