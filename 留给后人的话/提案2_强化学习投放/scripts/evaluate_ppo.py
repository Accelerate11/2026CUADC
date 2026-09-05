import argparse
import _bootstrap


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config')
    p.add_argument('--model', default='outputs/ppo_policy.zip')
    p.add_argument('--episodes', type=int, default=300)
    p.add_argument('--seed', type=int, default=82000000)
    p.add_argument('--out', default='outputs/ppo_evaluation.json')
    args = p.parse_args()
    if args.episodes < 1:
        p.error('--episodes must be positive')
    try:
        import numpy as np
        from stable_baselines3 import PPO
    except ImportError as error:
        p.exit(2, f'{error}\nInstall requirements-ppo.txt first.\n')
    from drop_rl.config import load_config
    from drop_rl.policies import Baseline
    from drop_rl.experiments import run_episode, summarize, write_json, write_csv
    cfg = load_config(args.config)
    model = PPO.load(args.model, device='cpu')

    class Policy:
        def act(self, obs, rng=None):
            a, _ = model.predict(np.asarray(obs.vector(), dtype=np.float32), deterministic=True)
            return int(a[2] > 0), tuple(float(v) * cfg.residual_limit_m_s for v in a[:2])

    all_rows, metrics = [], {}
    for name, policy in [('hover', Baseline('hover', cfg)), ('ballistic', Baseline('ballistic', cfg)), ('ppo', Policy())]:
        rows = [run_episode(policy, cfg, args.seed + i)[0] for i in range(args.episodes)]
        metrics[name] = summarize(rows)
        all_rows.extend({'policy': name, **r} for r in rows)
    write_json(args.out, {'config': cfg.to_dict(), 'seed_start': args.seed,
                          'seed_end': args.seed+args.episodes-1, 'metrics': metrics})
    write_csv(args.out + '.csv', all_rows)
    print(metrics)


if __name__ == '__main__':
    main()
