import json
import math
import random
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from drop_rl.config import Config
from drop_rl.env import DropEnv
from drop_rl.experiments import run_episode, summarize
from drop_rl.integration import ShadowSupervisor, observation_from_snapshot, rotate_xyzw
from drop_rl.observation import Observation, gate_reason, nominal_miss
from drop_rl.physics import ballistic_landing, drift, fall_time, first_order_axis
from drop_rl.policies import Baseline, QPolicy

ROOT = Path(__file__).resolve().parents[1]


class PhysicsTests(unittest.TestCase):
    def test_freefall_reference(self):
        self.assertAlmostEqual(fall_time(4.903325), 1.0)
        landing, t = ballistic_landing((1, -1), (2, -3), 4.903325)
        self.assertEqual(landing, (3.0, -4.0))

    def test_zero_drag_and_wind(self):
        self.assertAlmostEqual(drift(0.2, 100, 0, 1), 0.2)
        self.assertGreater(drift(0, 2, 0.5, 1), 0)
        self.assertAlmostEqual(drift(0.2, 0, 1e-12, 1), 0.2)

    def test_exact_vehicle_step_composition(self):
        p, v = first_order_axis(1, 0.2, 0.6, 0.1, 0.3, 0.2)
        p1, v1 = first_order_axis(1, 0.2, 0.6, 0.1, 0.3, 0.1)
        p2, v2 = first_order_axis(p1, v1, 0.6, 0.1, 0.3, 0.1)
        self.assertAlmostEqual(p, p2)
        self.assertAlmostEqual(v, v2)

    def test_invalid_geometry(self):
        for cfg in (replace(Config(), bucket_radius_m=0.02), replace(Config(), dt=0),
                    replace(Config(), dropout_probability=2), replace(Config(), kp=float('nan'))):
            with self.assertRaises(ValueError):
                cfg.validate()


class EnvironmentTests(unittest.TestCase):
    def test_repeatable_seed_and_actions(self):
        a, b = DropEnv(), DropEnv()
        self.assertEqual(a.reset(42), b.reset(42))
        for _ in range(30):
            self.assertEqual(a.step(0), b.step(0))

    def test_full_vision_loss_never_releases(self):
        cfg = replace(Config(), dropout_probability=1.0)
        env = DropEnv(cfg)
        env.reset(2)
        while not env.done:
            _, _, terminated, truncated, info = env.step(1)
        self.assertEqual(info['outcome'], 'timeout')
        self.assertIsNone(info['landing_xy'])
        self.assertTrue(terminated)
        self.assertFalse(truncated)

    def test_single_release_episode(self):
        cfg = Config()
        result, env = run_episode(Baseline('hover', cfg), cfg, 2026)
        self.assertIsNotNone(result['landing_xy'])
        with self.assertRaises(RuntimeError):
            env.step(1)

    def test_bounded_residual_command(self):
        env = DropEnv()
        env.reset(4)
        for _ in range(5):
            env.step(0, (100, -100))
            self.assertLessEqual(math.hypot(*env.command), env.cfg.max_command_m_s + 1e-12)

    def test_no_true_physics_in_observation(self):
        env = DropEnv()
        obs = env.reset(10)
        self.assertTrue(set(vars(obs)).isdisjoint({'wind', 'drag', 'delay', 'landing_xy', 'actual_port'}))
        sensor_state = env.sensor_rng.getstate()
        # Same public state with different hidden wind/drag has the same first frame.
        other = DropEnv()
        other.reset(10)
        other.wind = [99, -99]
        other.drag = 99
        self.assertEqual(env._sense(), other._sense())

    def test_guard_fails_closed(self):
        obs = Observation(0, 0, 0, 0, 1.2, .09, True, .1, .9, 1, 1, 0, 0)
        for bad in (replace(obs, ex=float('nan')), replace(obs, age=-1),
                    replace(obs, age=1), replace(obs, vx=2), replace(obs, visible=False)):
            self.assertTrue(gate_reason(bad, Config()))

    def test_delayed_release_changes_position(self):
        cfg = replace(Config(), wind_max_m_s=0, vision_noise_m=0, velocity_noise_m_s=0,
                      depth_noise_m=0, port_uncertainty_m=0, vision_delay_steps=0,
                      dropout_probability=0, height_jitter_m=0, delay_jitter_s=0,
                      start_distance_min_m=0, start_distance_max_m=0,
                      initial_speed_m_s=0, stable_required_s=0)
        env = DropEnv(cfg)
        obs = env.reset(1)
        old = list(env.position)
        _, _, _, _, info = env.step(1)
        self.assertAlmostEqual(info['release_s'], cfg.release_delay_s)
        self.assertNotEqual(info['release_xy'][0], old[0] + cfg.port_x_m)

    def test_metrics_include_nonrelease_failure(self):
        cfg = replace(Config(), dropout_probability=1)
        row, _ = run_episode(Baseline('hover', cfg), cfg, 0)
        summary = summarize([row])
        self.assertEqual(summary['hit_rate'], 0)
        self.assertEqual(summary['release_rate'], 0)
        self.assertIsNone(summary['p90_error_m_released_only'])

    def test_q_serialization_preserves_actions(self):
        cfg = Config()
        policy = QPolicy(cfg)
        env = DropEnv(cfg)
        obs = env.reset(1)
        for _ in range(20):
            nxt, r, done, trunc, _ = env.step(0)
            policy.update(obs, 0, r, nxt, done or trunc)
            obs = nxt
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.json'
            policy.save(path)
            restored = QPolicy.load(path)
            self.assertEqual(restored.table, policy.table)
            self.assertEqual(restored.act(obs), policy.act(obs))


class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.sample = json.loads((ROOT / 'examples/telemetry.jsonl').read_text().splitlines()[0])
        self.cfg = Config()

    def test_body_to_world_quaternion(self):
        x, y, z = rotate_xyzw([0, 0, math.sqrt(.5), math.sqrt(.5)], [1, 0, -1])
        self.assertAlmostEqual(x, 0)
        self.assertAlmostEqual(y, 1)
        self.assertAlmostEqual(z, -1)

    def test_port_offset_and_height(self):
        obs = observation_from_snapshot(self.sample, self.cfg)
        self.assertAlmostEqual(obs.ex, .02)
        self.assertAlmostEqual(obs.height, 1.2)

    def test_stale_odometry_and_wrong_frames(self):
        for update in ({'odom_stamp_s': 0}, {'velocity_frame': 'NED'},
                       {'body_to_enu_xyzw_at_exposure': [0, 0, 0, 0]},
                       {'target_valid': 'true'}, {'vision_stamp_s': 20}):
            with self.assertRaises(ValueError):
                observation_from_snapshot({**self.sample, **update}, self.cfg)

    def test_shadow_one_shot_and_override(self):
        sup = ShadowSupervisor(Baseline('ballistic', self.cfg), self.cfg)
        self.assertTrue(sup.decide(self.sample)['candidate_release'])
        next_sample = {**self.sample, 'now_s': 10.1, 'odom_stamp_s': 10.1}
        self.assertFalse(sup.decide(next_sample)['candidate_release'])
        sup = ShadowSupervisor(Baseline('ballistic', self.cfg), self.cfg)
        self.assertFalse(sup.decide({**self.sample, 'pilot_override': True})['candidate_release'])

    def test_target_switch_does_not_reset_latch(self):
        sup = ShadowSupervisor(Baseline('ballistic', self.cfg), self.cfg)
        sup.decide(self.sample)
        out = sup.decide({**self.sample, 'target_id': 'other'})
        self.assertFalse(out['candidate_release'])
        self.assertIn('target changed', out['reason'])


if __name__ == '__main__':
    unittest.main()
