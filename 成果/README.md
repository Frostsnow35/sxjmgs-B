# CUMCM2026 成果物

2026 年全国大学生数学建模竞赛（B 题：全向干扰源自动定位与清除）成果物总目录。
仓库已重整：**根目录只有 `LICENSE` 与 `成果/`**，其余内容全部从仓库中移除。

## 目录结构

```
LICENSE                             MIT 许可证
成果/
├─ AGENTS.md                        AI 工作区规则（建模纪律、协作与完成标准）
├─ README.md                        本索引
├─ 源代码/                         最终确定的求解与绘图代码
│  ├─ 问题三_定位与清除/
│  │  ├─ Q3_robot_solver_v6_最终运行版.py      实际参赛运行、产生 formal-p3-* 证据的版本
│  │  └─ Q3_robot_solver_v6_论文附录版.py      论文附录中收录的等价精简版
│  ├─ 问题四_滚动决策/
│  │  ├─ Q4_robot_solver_v7_最终运行版.py      实际参赛运行、产生 formal-p4-* 证据的版本
│  │  └─ Q4_robot_solver_v7_论文附录版.py      论文附录中收录的等价精简版
│  ├─ 问题二_图件生成/generate_q2_fig2_fig3.py
│  ├─ 绘图脚本/                    Q3/Q4 论文插图脚本
│  └─ 共用模块/                    localization_geometry.py 及其单元测试
├─ 论文终稿/数模国赛论文v10.docx      提交用论文终稿（v4/v5/v6/v8/v9 为历史版本，已归档）
├─ 文献笔记/                        检索笔记、BibTeX 与两篇核心文献原文/提取文本
│  ├─ CUMCM2022-2024_评阅要点_检索笔记.md
│  ├─ methods_q3_q4.bib
│  └─ Tokekar2013 / VanderHook2012 原文与正文提取
├─ 经验帖/                          写作与竞赛经验沉淀
│  ├─ CUMCM数模论文写作与竞赛研究经验手册.md
│  └─ 国赛优秀论文特征分析与参赛指南.md
└─ 题目与说明/                      B 题题面、附件、仿真器说明文档
```

## 归档位置

被移出仓库的全部内容（探索目录 `issue/B/Q3&Q4 - exploration/`、`distill/` 原始资料、`docs/` 模型合同与草案、
`data/`、`output/`、`logs/`、`tmp/`、`plugins/`、`q3-solution/`、历代论文 docx、支撑材料 jlog 运行证据、
旧版 `README.md`/`plan.md`/`todo.md`/`.gitignore` 等）保存在本机仓库外的：

```
E:\CUMCM2026_archive\原始仓库_20260912\
```

本次重整前的仓库主线另存于本地分支 `backup/local-main-20260912`，历史提交仍可通过 `git log` 追溯。
