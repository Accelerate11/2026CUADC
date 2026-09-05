import argparse
from pathlib import Path
import _bootstrap


def main():
    p = argparse.ArgumentParser(description='Optional PPO: alignment residual + release timing')
    p.add_argument('--config')
    p.add_argument('--steps', type=int, default=200000)
    p.add_argument('--seed', type=int, default=11)
    p.add_argument('--out', default='outputs/ppo_policy')
    args = p.parse_args()
    if args.steps <= 0:
        p.error('--steps must be positive')
    try:
        import torch
        import gymnasium
        import stable_baselines3
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_checker import check_env
        from stable_baselines3.common.monitor import Monitor
        from drop_rl.gym_env import ResidualDropEnv
    except ImportError as error:
        p.exit(2, f'PPO dependencies missing: {error}\nRun: python -m pip install -r requirements-ppo.txt\n')
    from drop_rl.config import load_config
    from drop_rl.experiments import write_json
    cfg = load_config(args.config)
    torch.set_num_threads(1)
    env = ResidualDropEnv(cfg)
    check_env(env)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    wrapped = Monitor(env, filename=args.out + '.monitor.csv')
    model = PPO('MlpPolicy', wrapped, device='cpu', seed=args.seed, verbose=1,
                n_steps=512, batch_size=64, n_epochs=5, gamma=0.98,
                learning_rate=3e-4, policy_kwargs={'net_arch': [64, 64]})
    model.learn(total_timesteps=args.steps)
    model.save(args.out)
    write_json(args.out + '.metadata.json', {'config': cfg.to_dict(), 'seed': args.seed,
               'requested_steps': args.steps, 'actual_steps': model.num_timesteps,
               'torch': torch.__version__, 'gymnasium': gymnasium.__version__,
               'stable_baselines3': stable_baselines3.__version__,
               'note': 'smoke training does not establish convergence or superiority'})
    wrapped.close()


if __name__ == '__main__':
    main()
