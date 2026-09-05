"""Policy-facing interface. Never contains simulated wind, drag or landing truth."""
from dataclasses import dataclass
import math
from .physics import fall_time, clamp_vector


@dataclass(frozen=True)
class Observation:
    ex: float  # observed target relative to nominal release port, level local axes
    ey: float
    vx: float  # aircraft velocity in the SAME axes, measured at control time
    vy: float
    height: float  # observed release-port to rim vertical gap
    radius: float  # assumed/calibrated usable mouth radius
    visible: bool
    age: float  # time since exposure, not time since callback
    confidence: float
    elapsed: float
    stable: float
    last_cmd_x: float
    last_cmd_y: float

    def vector(self):
        values = (self.ex / 2, self.ey / 2, self.vx, self.vy, self.height / 3,
                  self.radius, float(self.visible), self.age, self.confidence,
                  self.elapsed / 12, self.stable, self.last_cmd_x, self.last_cmd_y)
        return [max(-10.0, min(10.0, v)) for v in values]


def current_error(obs):
    # First-order exposure-latency compensation using available velocity only.
    return obs.ex - obs.vx * obs.age, obs.ey - obs.vy * obs.age


def nominal_miss(obs, cfg):
    ex, ey = current_error(obs)
    t = fall_time(max(0.0, obs.height)) + cfg.release_delay_s
    # No live wind measurement is assumed. This is an estimate, not the true landing.
    return obs.vx * t - ex, obs.vy * t - ey


def gate_reason(obs, cfg, include_dwell=True):
    numeric = [v for k, v in vars(obs).items() if k != 'visible']
    if not all(math.isfinite(v) for v in numeric):
        return 'invalid_observation'
    if not obs.visible or not 0 <= obs.age <= cfg.vision_timeout_s:
        return 'vision_stale'
    if not cfg.confidence_min <= obs.confidence <= 1:
        return 'low_confidence'
    if not cfg.release_height_min_m <= obs.height <= cfg.release_height_max_m:
        return 'height_out_of_range'
    if math.hypot(obs.vx, obs.vy) > cfg.release_speed_max_m_s:
        return 'moving_too_fast'
    if include_dwell and obs.stable + 1e-9 < cfg.stable_required_s:
        return 'stabilizing'
    return ''


def alignment_command(obs, cfg, residual=(0.0, 0.0)):
    if not obs.visible or obs.age > cfg.vision_timeout_s:
        return 0.0, 0.0
    ex, ey = current_error(obs)
    rx, ry = clamp_vector(*residual, cfg.residual_limit_m_s)
    return clamp_vector(cfg.kp * ex - cfg.kd * obs.vx + rx,
                        cfg.kp * ey - cfg.kd * obs.vy + ry,
                        cfg.max_command_m_s)
