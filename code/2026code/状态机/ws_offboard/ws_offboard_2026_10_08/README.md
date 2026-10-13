# 2026 CUADC 状态机工作区

此目录保存 V16 完整开源整理版，方便把原飞机代码、参数、模型和部署流程放在一起学习与复现。

| 版本 | 内容 |
| --- | --- |
| [ws_offboard_2026_10_11_v16](ws_offboard_2026_10_11_v16/README.md) | V16 完整任务、视觉时间同步、分段投放、全部模型、标定与测试工具 |

先进入版本目录阅读README。代码包名沿用cuadc_full_mission_20260803，运行命令均从版本工作区根目录执行。本版核心算法、任务参数和模型原样保留，采用AGPL-3.0-only；实际验证范围在版本目录docs/VALIDATION.md中记录。

合入原2026CUADC仓库时，将版本文件夹放入code/2026code/状态机/ws_offboard/，再把上表的条目加入原README。自动回归workflow放在此目录的.github/workflows/v16.yml，合入时需移到仓库根并调整工作区路径。
