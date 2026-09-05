import argparse
import hashlib
from pathlib import Path
import _bootstrap
from drop_rl.config import load_config
from drop_rl.policies import Baseline, QPolicy
from drop_rl.experiments import run_episode, summarize, write_csv, write_json


def main():
    p = argparse.ArgumentParser(description='Paired-seed held-out evaluation')
    p.add_argument('--config')
    p.add_argument('--model', default='outputs/q_policy.json')
    p.add_argument('--episodes', type=int, default=300)
    p.add_argument('--seed', type=int, default=80000000)
    p.add_argument('--out', default='outputs/evaluation')
    args = p.parse_args()
    if args.episodes < 1:
        p.error('--episodes must be positive')
    cfg = load_config(args.config)
    policies = {name: Baseline(name, cfg) for name in ('random', 'hover', 'ballistic')}
    policies['q_learning'] = QPolicy.load(args.model, cfg)
    summaries, all_rows = {}, []
    for name, policy in policies.items():
        rows = [run_episode(policy, cfg, args.seed + i)[0] for i in range(args.episodes)]
        summaries[name] = {**summarize(rows), 'unknown_q_decisions': policy.unknown}
        all_rows.extend({'policy': name, **row} for row in rows)
        print(f"{name:12s} hit={summaries[name]['hit_rate']:.1%} "
              f"release={summaries[name]['release_rate']:.1%} "
              f"mean_error={summaries[name]['mean_error_m_released_only']} "
              f"timeouts={summaries[name]['timeouts']}")
    report = {'config': cfg.to_dict(), 'seed_start': args.seed,
              'seed_end': args.seed + args.episodes - 1,
              'model_sha256': hashlib.sha256(Path(args.model).read_bytes()).hexdigest(),
              'metrics': summaries,
              'scope': 'simple simulation; not field validation or official scoring'}
    write_json(args.out + '.json', report)
    write_csv(args.out + '.csv', all_rows)


if __name__ == '__main__':
    main()
