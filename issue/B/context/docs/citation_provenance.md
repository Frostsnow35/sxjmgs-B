# 引用与数据来源记录（citation provenance）

> 本文初稿引用的公开文献仅用于说明代码中实际采用的算法实现或软件包；题设数据、模拟器演练数据、覆盖引理、MEC 收缩定理、网格清除保证均由本队推导或从题目附件获得，不由文献替代证明。

## 1. 题设与操作来源

| 来源 | 路径/标识 | 用途 |
| --- | --- | --- |
| 竞赛题目 B 题 | `B题.pdf` | 问题三、问题四的全部题设 |
| 模拟器使用说明 | `附件/1.docx` | 测试入口、计时规则、日志导出规则 |
| 模拟器通信接口说明 | `附件/2.docx` | HTTP+JSON 协议、`/measure`、`/clear` 语义 |
| 模拟器演练行为日志 | `JLOG/p3-tenTrial-1/*.jlog`、`JLOG/p4-tenTrial-1/*.jlog` | 演练身份与案例编码核验 |
| 最新问题三演练行为日志 | `JLOG/p3-tenTrial-2/*.jlog` | 2026-09-11 外环 1124 m 版本十连测；10 个日志文件的日志头均为 `problem_no=3` |
| 客户端脱敏动作记录 | `Q3&Q4 - exploration/robot_q3_round*.jsonl`、`robot_q4_round*.jsonl` | 逐轮动作、成功清除计数、虚拟时间 |
| 批处理汇总 | `Q3&Q4 - exploration/drill_batch_q3.jsonl`、`drill_batch_q4.jsonl` | 十轮批次统计 |
| 最新问题三批次客户端汇总 | `Q3&Q4 - exploration/drill_batch_q3.jsonl` 的最后 10 条记录 | p3-tenTrial-2 的逐轮成功数、测向次数、清除请求、虚拟时间与均值；总虚拟时间按详细记录求和 |
| 最新问题三批次控制台日志 | `Q3&Q4 - exploration/q3_10round_v5_console.out.log` | p3-tenTrial-2 的人工可读逐轮运行摘要 |
| 本地模拟器 | `Q3&Q4 - exploration/local_env.py` | 仅用于本地蒙特卡洛，不替代模拟器演练结果 |

## 2. 公开文献引用清单

### 问题三初稿使用的引用

- **[1]** WELZL E. Smallest enclosing disks (balls and ellipsoids)[C]//New Results and New Trends in Computer Science. Berlin, Heidelberg: Springer, 1991: 359–370. DOI: [10.1007/BFb0038202](https://doi.org/10.1007/BFb0038202).

> 使用位置：有限顶点集最小包围圆的随机增量算法实现，见 `geometry.py` 中 `min_enclosing_circle`。

- **[2]** SUTHERLAND I E, HODGMAN G W. Reentrant polygon clipping[J]. Communications of the ACM, 1974, 17(1): 32–42. DOI: [10.1145/360767.360802](https://doi.org/10.1145/360767.360802).

> 使用位置：凸可行域多边形逐半平面裁剪实现，见 `geometry.py` 中 `clip_polygon`。

- **[3]** HELD M, KARP R M. A dynamic programming approach to sequencing problems[J]. Journal of the Society for Industrial and Applied Mathematics, 1962, 10(1): 196–210. DOI: [10.1137/0110015](https://doi.org/10.1137/0110015).

> 使用位置：清除阶段首个动作点的最短哈密顿路径精确排序，见 `geometry.py` 中 `order_points_exact_tsp`；论文只引用其动态规划思想，不把该文献当作本题几何覆盖或清除正确性的证明。DOI、作者、刊名、卷期和页码已于 2026-09-12 通过 Crossref 元数据核验。

### 问题四初稿使用的引用

- **[1]** WELZL E. Smallest enclosing disks (balls and ellipsoids)[C]//New Results and New Trends in Computer Science. Berlin, Heidelberg: Springer, 1991: 359–370. DOI: [10.1007/BFb0038202](https://doi.org/10.1007/BFb0038202).

> 使用位置：同问题三，MEC 计算。

- **[2]** VIRTANEN P, GOMMERS R, OLIPHANT T E, et al. SciPy 1.0: Fundamental algorithms for scientific computing in Python[J]. Nature Methods, 2020, 17(3): 261–272. DOI: [10.1038/s41592-019-0686-2](https://doi.org/10.1038/s41592-019-0686-2).

> 使用位置：三角格巡测的离线核验，使用 `scipy.spatial.Delaunay` 与 `ConvexHull` 检查 42 个与目标圆盘相交的三角形及最长边；对应代码见 `survey_design.py`。

## 3. 不列入公开参考文献表、但需要可复核的内部来源

- 七点巡测覆盖引理：本队推导，数值核验见 `Q3&Q4 - exploration/survey_design.py`。
- 扇形 MEC 半径公式与 Q3 有限步定位定理：本队推导，数值核验见 `Q3&Q4 - exploration/COMPETITIVE_ADVANTAGE.md`。
- 三角格生成算法与定向半平面覆盖引理：本队推导，代码见 `Q3&Q4 - exploration/strategy.py` 的 `q4_survey_points()`，核验见 `survey_design.py`。
- 巡测路径与扫描顺序优化：本队计算，数据见 `Q3&Q4 - exploration/route_study.py` 与 `route_optimization_study.json`。当前 Q3 路线为 6744 m，行扫基线为 7868 m；按全扫描上限的巡测虚拟时间分别为 2181.8 s 与 2406.6 s。该结果只比较巡测阶段，不替代十连测整轮时间。
- 当前 `geometry.py` 的圆盘外近似实现：首个圆盘使用 180 边形，后续圆盘使用 90 边形；`expand=True` 时外近似的最远半径因子为 `sec²(π/n)`。论文中的 MEC 上界按首个 180 边形的实际因子核算。
- 十轮演练统计：来源于上述模拟器侧结果摘要与客户端脱敏记录，不将 `LocalEnv` 结果混入。
- `p3-tenTrial-2` 最新十连测的主统计：JLOG 负责确认演练题号和批次身份，逐轮成功响应、测向次数、清除请求和虚拟时间以 `drill_batch_q3.jsonl` 最后 10 条记录为准；控制台日志仅作人工核对。未将其未经跨目录配对的成功数改写成模拟器侧 `jammer_count`。

## 4. 使用约定

1. 任何公开文献只标注在“实现采用的算法/软件”处；几何保证性结论一律写作本文引理/定理并给出证明，不标注为文献结论。
2. 演练轮次必须按模拟器侧结果摘要的 `problem_no` 与时间配对，不能仅凭脚本文件名。
3. 正式测试开始前必须由操作者确认对应正式入口；正式测试结果与模拟演练结果分表呈现。
