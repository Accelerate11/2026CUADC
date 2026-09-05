"""A small deterministic-seeded 2D near-hover rescue drop environment."""
from collections import deque
from dataclasses import asdict
import math
import random
from .config import Config
from .observation import Observation, alignment_command, gate_reason
from .physics import first_order_axis, ballistic_landing


class DropEnv:
    """Actions: 0 wait, 1 request release; optional bounded velocity residual.

    Observations are noisy camera/odometry-like measurements. True state is used
    only by physics and the judge. Each episode has one fixed, preselected bucket.
    """
    def __init__(self, config=None, record=True):
        self.cfg = (config or Config()).validate()
        self.record = record
        self.done = True
        self.seed = None

    def reset(self, seed=0):
        self.seed = int(seed)
        # Independent streams preserve paired disturbances when actions differ.
        rng = random.Random(self.seed)
        self.sensor_rng = random.Random(self.seed + 2000000033)
        c = self.cfg
        angle = rng.uniform(-math.pi, math.pi)
        distance = rng.uniform(c.start_distance_min_m, c.start_distance_max_m)
        self.position = [distance * math.cos(angle), distance * math.sin(angle)]
        self.velocity = [rng.uniform(-c.initial_speed_m_s, c.initial_speed_m_s) for _ in range(2)]
        self.height = c.height_m + rng.uniform(-c.height_jitter_m, c.height_jitter_m)
        self.wind = [rng.uniform(-c.wind_max_m_s, c.wind_max_m_s) for _ in range(2)]
        self.tau = c.vehicle_tau_s + rng.uniform(-c.tau_jitter_s, c.tau_jitter_s)
        self.drag = max(0.0, c.horizontal_drag_s_inv + rng.uniform(-c.drag_jitter_s_inv, c.drag_jitter_s_inv))
        self.delay = max(0.0, c.release_delay_s + rng.uniform(-c.delay_jitter_s, c.delay_jitter_s))
        self.actual_port = (c.port_x_m + rng.uniform(-c.port_uncertainty_m, c.port_uncertainty_m),
                            c.port_y_m + rng.uniform(-c.port_uncertainty_m, c.port_uncertainty_m))
        self.t = 0.0
        self.steps = 0
        self.stable = 0.0
        self.command = (0.0, 0.0)
        self.frames = deque()
        self.last_frame = None
        self.trace = []
        self.done = False
        self.result = None
        self.obs = self._sense()
        self._record('reset')
        return self.obs

    def _sense(self):
        c, rng = self.cfg, self.sensor_rng
        blackout = c.blackout_start_s <= self.t < c.blackout_start_s + c.blackout_duration_s
        # Draw noise every frame even during dropout, keeping exogenous streams aligned.
        frame = (self.t, -self.position[0] - c.port_x_m + rng.gauss(0, c.vision_noise_m),
                 -self.position[1] - c.port_y_m + rng.gauss(0, c.vision_noise_m),
                 self.height + rng.gauss(0, c.depth_noise_m))
        received = rng.random() >= c.dropout_probability and not blackout
        self.frames.append((frame, received))
        if len(self.frames) > c.vision_delay_steps:
            candidate, valid = self.frames.popleft()
            if valid:
                self.last_frame = candidate
        vx, vy = [v + rng.gauss(0, c.velocity_noise_m_s) for v in self.velocity]
        if self.last_frame is None:
            stamp, ex, ey, height = self.t - 10, 0.0, 0.0, c.height_m
        else:
            stamp, ex, ey, height = self.last_frame
        age = max(0.0, self.t - stamp)
        visible = self.last_frame is not None and age <= c.vision_timeout_s + 1e-9
        obs = Observation(ex, ey, vx, vy, height, c.effective_radius_m, visible,
                          age, 0.9 if visible else 0.0, self.t, self.stable, *self.command)
        if gate_reason(obs, c, include_dwell=False):
            self.stable = 0.0
        elif self.steps > 0:
            self.stable += c.dt
        return Observation(**{**asdict(obs), 'stable': self.stable})

    def _advance(self, duration):
        for i in range(2):
            self.position[i], self.velocity[i] = first_order_axis(
                self.position[i], self.velocity[i], self.command[i],
                self.cfg.vehicle_wind_gain * self.wind[i], self.tau, duration)

    def _record(self, event):
        if not self.record:
            return
        self.trace.append({'t': self.t, 'x': self.position[0], 'y': self.position[1],
                           'vx': self.velocity[0], 'vy': self.velocity[1],
                           'event': event, 'observation': asdict(self.obs)})

    def step(self, action, residual=(0.0, 0.0)):
        if self.done:
            raise RuntimeError('episode ended: call reset before step')
        if action not in (0, 1):
            raise ValueError('action must be 0 (wait) or 1 (release)')
        if len(residual) != 2 or not all(math.isfinite(v) for v in residual):
            raise ValueError('residual must contain two finite numbers')
        c = self.cfg
        reason = gate_reason(self.obs, c) if action else ''
        self.command = alignment_command(self.obs, c, residual)
        if action and not reason:
            decision_t = self.t
            self._advance(self.delay)  # command held until the mechanism opens
            self.t += self.delay
            release_xy = tuple(self.position[i] + self.actual_port[i] for i in range(2))
            landing, fall_s = ballistic_landing(release_xy, self.velocity, self.height,
                                               self.wind, self.drag)
            error = math.hypot(*landing)
            hit = error <= c.effective_radius_m
            reward = (8.0 if hit else -4.0) - min(12.0, (error / c.effective_radius_m)**2)
            self.done = True
            self.result = {'outcome': 'hit' if hit else 'miss', 'hit': hit,
                           'error_m': error, 'landing_xy': list(landing),
                           'release_xy': list(release_xy), 'decision_s': decision_t,
                           'release_s': self.t, 'fall_s': fall_s,
                           'seed': self.seed, 'blocked_reason': ''}
            self._record('release')
            return self.obs, reward, True, False, dict(self.result)
        self._advance(c.dt)
        self.steps += 1
        self.t = self.steps * c.dt
        self.obs = self._sense()
        self._record('blocked:' + reason if reason else 'wait')
        reward = -c.step_cost_per_s * c.dt - (0.05 if action and reason else 0.0)
        outside = math.hypot(*self.position) > c.fence_radius_m
        # Finite mission deadline is part of the state and is an MDP terminal failure.
        timed_out = self.t + 1e-9 >= c.horizon_s
        if outside or timed_out:
            self.done = True
            self.result = {'outcome': 'out_of_bounds' if outside else 'timeout',
                           'hit': False, 'error_m': None, 'landing_xy': None,
                           'release_xy': None, 'decision_s': None, 'release_s': None,
                           'fall_s': None, 'seed': self.seed, 'blocked_reason': reason}
            return self.obs, reward - 8.0, True, False, dict(self.result)
        return self.obs, reward, False, False, {'blocked_reason': reason}
