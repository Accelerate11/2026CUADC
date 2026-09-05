"""Explicit, validated demonstration parameters; none are flight calibration."""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class Config:
    dt: float = 0.1
    horizon_s: float = 12.0
    height_m: float = 1.2  # release port above bucket rim, not takeoff altitude
    height_jitter_m: float = 0.25
    bucket_radius_m: float = 0.125  # upstream scene: 25 cm diameter bucket
    payload_radius_m: float = 0.03  # illustrative effective bottle cross-section
    rim_margin_m: float = 0.005
    start_distance_min_m: float = 0.4
    start_distance_max_m: float = 1.4
    initial_speed_m_s: float = 0.2
    vehicle_tau_s: float = 0.35
    tau_jitter_s: float = 0.1
    max_command_m_s: float = 0.6
    residual_limit_m_s: float = 0.15
    kp: float = 0.9
    kd: float = 0.55
    wind_max_m_s: float = 1.2
    vehicle_wind_gain: float = 0.08
    horizontal_drag_s_inv: float = 0.18
    drag_jitter_s_inv: float = 0.06
    release_delay_s: float = 0.18
    delay_jitter_s: float = 0.07
    port_x_m: float = 0.08
    port_y_m: float = 0.0
    port_uncertainty_m: float = 0.01
    vision_noise_m: float = 0.018
    velocity_noise_m_s: float = 0.012
    depth_noise_m: float = 0.02
    vision_delay_steps: int = 1
    dropout_probability: float = 0.04
    blackout_start_s: float = -1.0
    blackout_duration_s: float = 0.0
    vision_timeout_s: float = 0.3
    confidence_min: float = 0.5
    release_speed_max_m_s: float = 0.25
    release_height_min_m: float = 0.5
    release_height_max_m: float = 3.0
    stable_required_s: float = 0.2
    fence_radius_m: float = 3.0
    step_cost_per_s: float = 0.25

    @property
    def effective_radius_m(self):
        return self.bucket_radius_m - self.payload_radius_m - self.rim_margin_m

    def validate(self):
        for key, value in asdict(self).items():
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
                raise ValueError(f"{key} must be a finite number")
        positive = ('dt', 'horizon_s', 'height_m', 'bucket_radius_m', 'vehicle_tau_s',
                    'max_command_m_s', 'vision_timeout_s', 'release_speed_max_m_s',
                    'release_height_min_m', 'fence_radius_m')
        for key in positive:
            if getattr(self, key) <= 0:
                raise ValueError(f"{key} must be positive")
        for key, value in asdict(self).items():
            if key != 'blackout_start_s' and key not in ('port_x_m', 'port_y_m') and value < 0:
                raise ValueError(f"{key} must be nonnegative")
        if not 0 <= self.dropout_probability <= 1 or not 0 <= self.confidence_min <= 1:
            raise ValueError('probabilities must lie in [0, 1]')
        if type(self.vision_delay_steps) is not int or self.vision_delay_steps < 0:
            raise ValueError('vision_delay_steps must be a nonnegative integer')
        if self.vision_delay_steps > 1000:
            raise ValueError('vision_delay_steps is unreasonably large')
        if self.effective_radius_m <= 0:
            raise ValueError('bucket opening must exceed payload plus margin')
        if self.vehicle_tau_s <= self.tau_jitter_s:
            raise ValueError('vehicle_tau_s must exceed tau_jitter_s')
        if self.height_m <= self.height_jitter_m:
            raise ValueError('height must stay positive')
        if self.start_distance_min_m > self.start_distance_max_m:
            raise ValueError('invalid initial distance range')
        if self.release_height_min_m >= self.release_height_max_m:
            raise ValueError('invalid release height range')
        if self.dt > self.horizon_s:
            raise ValueError('dt exceeds horizon')
        return self

    def to_dict(self):
        return asdict(self)


def load_config(path=None):
    if path is None:
        return Config().validate()
    values = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    return Config(**values).validate()
