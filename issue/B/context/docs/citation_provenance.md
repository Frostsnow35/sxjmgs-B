# 引用与数据来源记录（citation provenance）

> 更新日期：2026-09-11。公开文献只用于说明代码中实际采用的算法实现或软件包；题设数据、模拟器演练数据、双环覆盖引理、MEC 收缩定理、联合投影模型与网格清除保证均由本队推导或从题目附件获得，不由文献替代证明。

## 1. 题设与操作来源

| 来源 | 路径/标识 | 用途 |
| --- | --- | --- |
| 竞赛题目 B 题 | `B题.pdf` | 问题三、问题四的题设与边界条件 |
| 模拟器使用说明 | `附件/1.docx` | 演练入口、计时规则、日志导出规则 |
| 模拟器通信接口说明 | `附件/2.docx` | HTTP+JSON 协议、`/measure`、`/clear` 语义 |
| 模拟器演练行为日志 | `JLOG/p3-tenTrial-1/*.jlog`、`JLOG/p4-tenTrial-1/*.jlog` | 演练身份与案例编码核验 |
| 客户端脱敏动作记录 | `Q3&Q4 - exploration/robot_q3_round*.jsonl`、`robot_q4_round*.jsonl` | 逐轮动作、成功清除计数、虚拟时间 |
| 批处理汇总 | `Q3&Q4 - exploration/drill_batch_q3.jsonl`、`drill_batch_q4.jsonl` | 十轮批次统计 |
| 本地模拟器 | `Q3&Q4 - exploration/local_env.py` | 仅用于本地蒙特卡洛，不替代模拟器演练结果 |

## 2. 公开文献引用清单

### 问题三初稿使用的引用

[1] WELZL E. Smallest enclosing disks (balls and ellipsoids)[C]//New Results and New Trends in Computer Science. Berlin, Heidelberg: Springer, 1991: 359-370. DOI: 10.1007/BFb0038202.

> 使用位置：有限顶点集最小包围圆的随机增量算法实现，见 `geometry.py` 中 `min_enclosing_circle`。

[2] SUTHERLAND I E, HODGMAN G W. Reentrant polygon clipping[J]. Communications of the ACM, 1974, 17(1): 32-42. DOI: 10.1145/360767.360802.

> 使用位置：凸可行域多边形逐半平面裁剪实现，见 `geometry.py` 中 `clip_polygon`。

### 问题四初稿使用的引用

[1] SUTHERLAND I E, HODGMAN G W. Reentrant polygon clipping[J]. Communications of the ACM, 1974, 17(1): 32-42. DOI: 10.1145/360767.360802.

> 使用位置：保守凸可行域的逐半平面多边形裁剪，见 `geometry.py` 中 `clip_polygon`。

[2] VIRTANEN P, GOMMERS R, OLIPHANT T E, et al. SciPy 1.0: Fundamental algorithms for scientific computing in Python[J]. Nature Methods, 2020, 17(3): 261-272. DOI: 10.1038/s41592-019-0686-2.

> 使用位置：25 点双环三角剖分的离线核验，使用 `scipy.spatial.Delaunay` 与 `ConvexHull`，对应 `survey_design.py`。

[3] WELZL E. Smallest enclosing disks (balls and ellipsoids)[C]//New Results and New Trends in Computer Science. Berlin, Heidelberg: Springer, 1991: 359-370. DOI: 10.1007/BFb0038202.

> 使用位置：有限顶点集 MEC 计算，见 `geometry.py` 中 `min_enclosing_circle`。

[4] ANDREW A M. Another efficient algorithm for convex hulls in two dimensions[J]. Information Processing Letters, 1979, 9(5): 216-219. DOI: 10.1016/0020-0190(79)90072-3.

> 使用位置：正信号点凸包的单调链构造，见 `joint_feasibility.py` 中 `_convex_hull`。DOI、作者、刊名、卷期和页码已于 2026-09-11 通过 Crossref 元数据核验。

## 3. 不列入公开参考文献表、但需要可复核的内部来源

- 七点巡测覆盖引理：本队推导，数值核验见 `Q3&Q4 - exploration/survey_design.py`。
- 扇形 MEC 半径公式与 Q3 有限步定位定理：本队推导，数值核验见 `Q3&Q4 - exploration/COMPETITIVE_ADVANTAGE.md`。
- 双环三角剖分生成与定向半平面覆盖引理：本队推导，代码见 `Q3&Q4 - exploration/strategy.py` 的 `q4_survey_points()`，核验见 `survey_design.py`。
- 位置—方向联合可行性模型、$r_{\min}$ 消元、半圆弧可行性判定和凸包内负测量安全推论：本队针对题设的推导，代码见 `joint_feasibility.py`；不把外部算法文献作为其正确性证明。
- 巡测路径与扫描顺序优化：本队计算；只有与当前巡测集合、起始点和扫描规则一致的结果才可写入论文。
- 十轮演练统计：来源于上述模拟器侧结果摘要与客户端脱敏记录，不将 `LocalEnv` 结果混入。

## 4. 使用约定

1. 公开文献只标注在“实现采用的算法/软件”处；几何保证性结论一律写作本文引理/定理并给出证明，不标注为文献结论。
2. 演练轮次必须按模拟器侧结果摘要的 `problem_no` 与时间配对，不能仅凭脚本文件名。
3. 正式测试开始前必须由操作者确认对应正式入口；正式测试结果与模拟演练结果分表呈现。
