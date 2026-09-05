"""Optional Gymnasium adapter; requires requirements-ppo.txt."""
import gymnasium as gym
import numpy as np
from .env import DropEnv


class ResidualDropEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, config=None):
        super().__init__()
        self.core = DropEnv(config, record=False)
        self.observation_space = gym.spaces.Box(-10.0, 10.0, shape=(13,), dtype=np.float32)
        # x/y velocity residual, release request logit (positive => request)
        self.action_space = gym.spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if options:
            raise ValueError('reset options are not supported')
        episode_seed = int(self.np_random.integers(0, 2**31-1))
        obs = self.core.reset(episode_seed)
        return np.asarray(obs.vector(), dtype=np.float32), {}

    def step(self, action):
        a = np.asarray(action, dtype=np.float32)
        if a.shape != (3,) or not np.isfinite(a).all():
            raise ValueError('expected three finite actions')
        a = np.clip(a, -1.0, 1.0)
        residual = tuple(float(v) * self.core.cfg.residual_limit_m_s for v in a[:2])
        obs, reward, terminated, truncated, info = self.core.step(int(a[2] > 0), residual)
        # The base alignment loop stays active. Residuals have an effort cost.
        reward -= 0.01 * float(np.dot(a[:2], a[:2]))
        return np.asarray(obs.vector(), dtype=np.float32), reward, terminated, truncated, info
