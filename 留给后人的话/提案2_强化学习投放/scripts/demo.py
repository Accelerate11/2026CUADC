import argparse
import _bootstrap
from drop_rl.config import load_config
from drop_rl.policies import Baseline, QPolicy
from drop_rl.experiments import run_episode, write_json
from drop_rl.replay import export_html


def main():
    p = argparse.ArgumentParser(description='One drop episode with offline HTML replay')
    p.add_argument('--policy', choices=['hover', 'ballistic', 'random', 'q'], default='hover')
    p.add_argument('--config')
    p.add_argument('--model', default='outputs/q_policy.json')
    p.add_argument('--seed', type=int, default=2026)
    p.add_argument('--out', default='outputs/demo.html')
    args = p.parse_args()
    cfg = load_config(args.config)
    policy = QPolicy.load(args.model, cfg) if args.policy == 'q' else Baseline(args.policy, cfg)
    result, env = run_episode(policy, cfg, args.seed, record=True)
    export_html(env, args.out, args.policy)
    write_json(args.out + '.json', {'config': cfg.to_dict(), 'result': result, 'trace': env.trace})
    print(result)
    print(f'Open {args.out} in a browser.')


if __name__ == '__main__':
    main()
