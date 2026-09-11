# Q3&Q4 探索实验目录

本目录保存 B 题第三问与第四问的策略探索、本地演练产物和模拟器客户端。

## 文件清单

| 文件 | 作用 |
| --- | --- |
| `SOLUTION_NOTES.md` | Q2/Q3/Q4 数学模型、证明口径与适用边界 |
| `strategy.py` | Q3/Q4 自动搜索、定位与清除策略 |
| `geometry.py` | 角度误差锥、可行域多边形、最小包围圆、网格覆盖等几何工具 |
| `joint_feasibility.py` | Q4 位置—朝向联合可行性判定、凸包内安全负测量与位置投影 |
| `evaluate_joint_projection.py` | 仅 LocalEnv 的固定种子配对评估，不连接模拟器 |
| `local_env.py` | 按题目与附件规则编写的本地模拟器（非官方模拟器） |
| `monte_carlo_test.py` | 本地蒙特卡洛演练脚本 |
| `run_safe_rehearsal.py` | **仅演练**的安全入口；需显式人工确认 UI，绝不提供正式测试选项 |
| `robot_client.py`、`run_drill_batch*.py` | 历史运行器与批处理记录，仅作复盘；不得作为后续执行入口 |
| `build_evidence_manifest.py` | 从自留 JSONL、模拟器结果侧文件和 JLOG 重建脱敏证据清单，不连接模拟器 |
| `evidence_manifest.json` | 已冻结的批次配对、分类和统计结果；不含队号、票据、密钥或案例编码 |
| `survey_design.py` | 巡测点设计验证与地图绘制 |
| `q2_analysis.py` | 第二问第二检测点选点数值分析 |
| `monte_carlo_q3.json` | Q3 本地演练 100 个案例结果 |
| `monte_carlo_q4.json` | Q4 本地演练 100 个案例结果 |
| `*.png` | 巡测点地图与 Q2 候选区域热力图 |

## 冻结的证据口径

`evidence_manifest.json` 以每轮客户端日志的 `/exit` UTC 时刻配对模拟器侧
`*.result.json`，而不是仅按脚本文件名分类。表中的源数来自模拟器侧结果摘要，
成功响应数来自客户端脱敏日志；两者是本地归档的配对一致性核验，而非正式测试证书。

| 类别 | 可计入轮数 | 模拟器侧源数 / 客户端成功响应 | 逐轮计数一致性 | 总虚拟时间 / 源数 | 虚拟总时间均值 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Q3 模拟器演练 | 10 | 131 / 131 | 10/10 | 410.35 s/源 | 5375.57 s |
| Q4 模拟器演练 | 9 | 116 / 116 | 9/9 | 915.67 s/源 | 11801.97 s |

“总虚拟时间 / 源数”取所有已计入轮次的虚拟总时间除以相应模拟器侧源数；它包含巡测、
移动、频道切换、检测和清除，不能解释为单一局部定位步骤的耗时。`robot_q4_round01.jsonl`
的脚本模式虽为 Q4，但它按时间戳配对到实际问题三演练，故标为 `mode_mismatch`，不计入
Q4 表格。上述均为演练，不是正式测试；原始 JLOG 与 JSONL 均不提交。

## 本地演练结果（非官方 LocalEnv）

| 问 | 案例数 | 本地环境完成比例 | 总虚拟时间 / 成功源数 | 虚拟总时间均值 | 请求数均值/最大 |
| --- | --- | --- | --- | --- | --- |
| Q3 | 100 | 100% | 409.0 s | 5131.7 s | 156.4 / 177 |
| Q4 | 100 | 100% | 589.1 s | 7358.6 s | 319.1 / 410 |

这些是按固定种子在自建 LocalEnv 中的回放结果，只验证本地实现，不能替代模拟器演练。
其中 `joint_projection_localenv.json` 的 50 组开启/关闭联合排序配对均清除 646 个源；平均虚拟时间差为 $+0.056$ s/轮，故它只能说明排序没有破坏本地清除完整性，不能作为提速结论。
复现命令：

```powershell
python monte_carlo_test.py --mode q3 --cases 100 --seed 1000
python monte_carlo_test.py --mode q4 --cases 100 --seed 2000
python survey_design.py
python q2_analysis.py
```

## 后续演练的安全步骤

1. 由操作者在模拟器中手动选择并确认 **问题3演练测试** 或 **问题4演练测试**；本目录脚本不会判断或切换 UI。
2. 等待倒计时结束、界面显示接口就绪后，在本目录运行：

```powershell
python run_safe_rehearsal.py --mode q3 --robot-id <参赛队号> --rehearsal-confirmed --output <脱敏汇总路径> --log <本地审计日志路径>
python run_safe_rehearsal.py --mode q4 --robot-id <参赛队号> --rehearsal-confirmed --output <脱敏汇总路径> --log <本地审计日志路径>
```

3. 未传 `--rehearsal-confirmed` 时入口只写拒绝摘要，不构造客户端、不探测端口、不调用 `/enter`。每个业务动作均有 HTTP/`accepted` 双校验、最多三次网络尝试和真实时间余量门禁。
4. 原始 JLOG 的信封含身份关联字段，只在本地按原名保存为支撑材料；团队仓库仅提交 `evidence_manifest.json` 等脱敏汇总。

> 注意：本目录不提供正式测试执行路径。问题三、四的正式测试均须在当次另获用户明确授权后，使用独立冻结版本和证据目录处理。
