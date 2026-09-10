# CUMCM 分阶段提示词

以下提示词按比赛时间顺序排列。正式比赛时可整段复制，也可只保留与当前阶段相关的段落。每个 Phase 都以“先读 `AGENTS.md` + 相关技能”为开头，避免模型脱离项目规范。

## Phase 0 - 赛前环境与分工

```text
请先读取项目 AGENTS.md 和 prompts/cumcm-phased-prompts.md。
当前没有赛题时，不要展开建模。请确认：
1. Python 环境与 requirements.txt 是否可运行；
2. data/raw、data/processed、src、scripts、output/results、output/figures、logs 是否已建好；
3. 团队角色：谁负责题目理解、谁负责模型、谁负责代码、谁负责论文；
4. 允许使用的工具、数据来源与 AI 披露要求，以官方通知为准；
5. 给每个角色生成一份“比赛开始后前 3 小时”的行动清单。
```

## Phase 1 - 开题 0-3 小时：题目拆解与选题

```text
请读取题面，不要先想论文。
输出一份 PROBLEM_BRIEF.md，包含：
- 赛题类型预判（A 机理型 / B 运筹优化型 / 混合型）；
- 子问题编号、每问的输入、输出、评价口径；
- 附件数据清单、缺失项、单位与异常风险；
- 2-3 个必须由团队确认的歧义点；
- 若还有 C/D 题候选，比较每题的数据量与建模难度。
完成后询问我：选择哪题、采用哪个歧义解释，再进入 Phase 2。
```

## Phase 2 - Day 1：机理/优化建模设计

```text
我选择题目为 {题目}，歧义解释见 PROBLEM_BRIEF.md。
请根据题目类型：
- A 题：调用 $cumcm-a-mechanism-modeling；
- B 题：调用 $cumcm-b-optimization；
- 混合题：逐问调用对应技能。
只做建模设计，不写论文正文。产出：
- assumptions.md：逐条假设及违反后果；
- symbols.md：变量、单位、公式编号；
- models.md：每个子问题的数学表述与求解策略；
- todo.md：下一阶段代码任务与验证计划。
```

## Phase 3 - Day 1-2：数据与代码求解

```text
请按 todo.md 执行。规则：
- 原始数据只放入 data/raw，只读；清洗结果写入 data/processed；
- 求解脚本写入 scripts/，模块放入 src/；
- 随机算法固定 SEED，运行命令和结果路径写入 logs/；
- 每个子问题先做小规模正确性验证，再跑正式数据；
- 每完成一个脚本运行一次，并汇报退出状态、关键数值与运行耗时；
- 若结果不可行/不收敛/误差过大，先诊断，不得直接改结果文件。
```

## Phase 4 - Day 2：结果冻结与图表

```text
请将当前已核实的最终数值冻结到 output/results/final_results.json。
之后：
- 生成每个子问题的结果表与论文图；
- 图中标注单位、关键数值与来源脚本；
- 对主结果做一次灵敏度或稳健性分析并记录；
- 任何尚未核实的数字不得进入 frozen results；
- 输出 frozen_results.json、figures 清单、与每个结论的溯源表。
```

## Phase 5 - Day 3：论文写作与一致性

```text
请以 output/results/final_results.json 为唯一数值来源撰写论文正文。
写作时调用 $research-writing-skill；公式用 Markdown/LaTeX 保持可追踪。
完成后执行一致性检查：
- 正文每个数字与 frozen results 一致；
- 图表编号、单位、图题完整；
- 每个模型都有假设、求解方法、结果、检验、局限性；
- 不写任何没有来源的参考文献或外部数据；
- 给我一份“仍有疑问的写作决策清单”。
```

## Phase 6 - 提交前审计

```text
请执行最终审计，不要修改核心结果。
先运行 $bzd-paper-aigc-auditor 检查论文风险与模型合理性；
再人工核对：
1. 是否回答了每个子问题的每项要求；
2. 模型能否用一句话说清、代码能否复现；
3. 关键结果是否全部来自 final_results.json；
4. 是否遗漏假设、符号表、附录、数据来源或 AI 使用声明；
5. 按当届官方要求检查提交格式与截止时间。
最终输出 AUDIT_REPORT.md：问题清单、严重度、修复建议、剩余风险。
```

## 用时约束建议

- Phase 1 不超过 3 小时；
- Phase 2 与 Phase 3 的核心模型/首版可运行代码应在 Day 1 结束前完成；
- Phase 4 应在 Day 2 白天完成，给 Day 3 留足写作与审计时间；
- 任何阶段若连续卡住超过 1 小时，要求 Agent 改用更简单但可辩护的替代方案，而不是继续“加复杂度”。
