import importlib.util
import unittest


@unittest.skipUnless(importlib.util.find_spec('gymnasium') and importlib.util.find_spec('stable_baselines3'),
                     'optional PPO dependencies are not installed')
class GymTests(unittest.TestCase):
    def test_sb3_checker_and_seed(self):
        import numpy as np
        from stable_baselines3.common.env_checker import check_env
        from drop_rl.gym_env import ResidualDropEnv
        env = ResidualDropEnv()
        check_env(env)
        a, _ = env.reset(seed=123)
        b, _ = env.reset(seed=123)
        np.testing.assert_array_equal(a, b)
        for _ in range(20):
            obs, reward, done, truncated, _ = env.step(np.array([0., 0., -1.], dtype=np.float32))
            self.assertTrue(env.observation_space.contains(obs))
            if done or truncated:
                env.reset()


if __name__ == '__main__':
    unittest.main()
