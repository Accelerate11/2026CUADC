# 本次运行快照

这是 2026-09-05 对本草案实际运行后保存的结果，不是人工编造的演示数据。完整结论与实验口径在 [运行结果](../../docs/04_实验结果.md)。

- 直接打开 [Q-learning 回放](demo_q.html) 或 [悬停基线回放](demo_hover.html)，无网络和安装要求。
- [默认场景结果](evaluation.json)、[压力场景结果](stress.json) 及相应 CSV 可逐场核对。
- `q_policy.json` 是 6000 回合训练的最终 Q 表，可直接传给脚本 `--model`。
- `ppo_smoke.zip` 是约 3 万步的 PPO 运行验证模型，使用它需要可选依赖，不代表已收敛。
- `manifest.json` 为这些快照提供 SHA-256；`not_generated` 列出运行时未生成的可选产物。
- `replay_preview.png` 为无界面浏览器检查时截取的桌面回放界面。

训练日志中的探索命中率与评估时关闭探索的命中率不是同一口径。压力场景与小桶含未投放回合，详见释放率和超时字段。

从项目根目录运行归档模型：

```bash
python scripts/demo.py --policy q --model examples/results/q_policy.json --out outputs/from_saved_model.html
```
