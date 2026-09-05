import argparse
import random
import time
import _bootstrap
from drop_rl.config import load_config
from drop_rl.env import DropEnv
from drop_rl.policies import QPolicy
from drop_rl.experiments import write_csv


def main():
    p = argparse.ArgumentParser(description='Standard-library Q-learning release timing')
    p.add_argument('--config')
    p.add_argument('--episodes', type=int, default=6000)
    p.add_argument('--seed', type=int, default=11)
    p.add_argument('--out', default='outputs/q_policy.json')
    args = p.parse_args()
    if args.episodes < 1:
        p.error('--episodes must be positive')
    cfg = load_config(args.config)
    policy, env = QPolicy(cfg), DropEnv(cfg, record=False)
    rng = random.Random(args.seed)
    rows, start = [], time.perf_counter()
    for episode in range(args.episodes):
        obs = env.reset(args.seed * 1000000 + episode)
        epsilon = max(0.04, 0.7 * (1 - episode / args.episodes))
        total = 0.0
        while not env.done:
            action = policy.act(obs, rng, epsilon)
            nxt, reward, terminated, truncated, info = env.step(action)
            policy.update(obs, action, reward, nxt, terminated or truncated)
            obs, total = nxt, total + reward
        rows.append({'episode': episode, 'return': total, 'hit': int(info['hit']),
                     'epsilon': epsilon, 'outcome': info['outcome']})
        if (episode + 1) % 500 == 0:
            recent = rows[-500:]
            print(f"episode={episode+1} exploratory_hit_rate={sum(r['hit'] for r in recent)/len(recent):.3f}", flush=True)
    elapsed = time.perf_counter() - start
    policy.save(args.out, {'algorithm': 'tabular Q-learning', 'episodes': args.episodes,
                          'seed': args.seed, 'train_seed_start': args.seed * 1000000,
                          'train_seed_end': args.seed * 1000000 + args.episodes - 1,
                          'alpha': 0.15, 'gamma': 0.98, 'wall_s': elapsed,
                          'note': 'simulation only; no validation-set model selection'})
    write_csv(str(args.out) + '.training.csv', rows)
    print(f'Saved {args.out}; {len(policy.table)} states; {elapsed:.1f} s')


if __name__ == '__main__':
    main()
