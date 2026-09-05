"""Read-only/shadow integration contract for the existing mission state machine.

No ROS import, flight-control publisher or servo command exists in this module.
The existing mission must provide a stable target ID and synchronized observations.
"""
from dataclasses import dataclass
import math
from .observation import Observation, alignment_command, gate_reason


def rotate_xyzw(q, vector):
    """Quaternion must map body FLU to local ENU at camera exposure time."""
    if len(q) != 4 or len(vector) != 3 or not all(math.isfinite(v) for v in (*q, *vector)):
        raise ValueError('invalid quaternion/vector')
    norm = math.sqrt(sum(v*v for v in q))
    if abs(norm - 1.0) > 0.01:
        raise ValueError('quaternion must be normalized')
    x, y, z, w = [v/norm for v in q]
    a, b, c = vector
    tx, ty, tz = 2*(y*c-z*b), 2*(z*a-x*c), 2*(x*b-y*a)
    return a+w*tx+y*tz-z*ty, b+w*ty+z*tx-x*tz, c+w*tz+x*ty-y*tx


def observation_from_snapshot(data, cfg):
    """target_body_flu is already calibrated from the existing RGB-D camera.

    Position and orientation correspond to vision_stamp_s; ENU velocity is from
    odom_stamp_s. Exposure age is retained for first-order latency compensation.
    """
    if data['body_frame'] != 'FLU' or data['velocity_frame'] != 'ENU':
        raise ValueError('explicit FLU body and ENU velocity frames are required')
    numeric = ('now_s', 'vision_stamp_s', 'odom_stamp_s', 'confidence', 'elapsed_s', 'stable_s')
    if not all(math.isfinite(data[k]) for k in numeric):
        raise ValueError('nonfinite timestamp/quality value')
    if not 0 <= data['now_s'] - data['odom_stamp_s'] <= 0.2:
        raise ValueError('odometry stale or future-dated')
    age = data['now_s'] - data['vision_stamp_s']
    if age < 0:
        raise ValueError('vision timestamp is in the future')
    if data['elapsed_s'] < 0 or data['stable_s'] < 0:
        raise ValueError('negative mission time')
    target, port = data['target_body_flu_m'], data['release_port_body_flu_m']
    if len(target) != 3 or len(port) != 3:
        raise ValueError('target and port must be 3D vectors')
    relative = [target[i] - port[i] for i in range(3)]
    ex, ey, ez = rotate_xyzw(data['body_to_enu_xyzw_at_exposure'], relative)
    velocity, command = data['velocity_enu_m_s'], data['last_command_enu_m_s']
    if len(velocity) != 3 or len(command) != 2 or not all(math.isfinite(v) for v in (*velocity, *command)):
        raise ValueError('invalid velocity/command')
    if abs(velocity[2]) > 0.20:
        raise ValueError('vertical speed exceeds near-hover model scope')
    if type(data['target_valid']) is not bool:
        raise ValueError('target_valid must be boolean')
    # The simulator's height is constant. A deployed implementation must maintain it
    # using the mission controller and validate exposure-time motion compensation.
    return Observation(ex, ey, velocity[0], velocity[1], -ez, cfg.effective_radius_m,
                       data['target_valid'], age, data['confidence'], data['elapsed_s'],
                       data['stable_s'], command[0], command[1])


@dataclass
class ShadowSupervisor:
    policy: object
    cfg: object
    issued: bool = False
    active_key: tuple | None = None
    last_now: float | None = None

    def decide(self, snapshot):
        out = {'mode': 'shadow_only', 'candidate_release': False,
               'candidate_velocity_enu_m_s': [0.0, 0.0], 'reason': ''}
        try:
            key = (str(snapshot['mission_id']), str(snapshot['target_id']), str(snapshot['payload_id']))
            if any(not item for item in key):
                raise ValueError('mission/target/payload IDs are required')
            # The example handles exactly one latched mission target/payload. Recreate
            # only on an explicit next-target transition in the host state machine.
            if self.active_key is None:
                self.active_key = key
            elif self.active_key != key:
                raise ValueError('target changed: host mission must reset supervisor explicitly')
            for flag in ('connected', 'armed', 'guided', 'pilot_override'):
                if type(snapshot[flag]) is not bool:
                    raise ValueError(f'{flag} must be boolean')
            if self.last_now is not None and snapshot['now_s'] <= self.last_now:
                raise ValueError('duplicate or backwards control timestamp')
            self.last_now = snapshot['now_s']
            if not (snapshot['connected'] and snapshot['armed'] and snapshot['guided']) or snapshot['pilot_override']:
                raise ValueError('mission inactive or pilot override')
            if snapshot['stage'] not in ('ALIGN', 'RELEASE'):
                raise ValueError('outside drop phase')
            obs = observation_from_snapshot(snapshot, self.cfg)
            if obs.elapsed >= self.cfg.horizon_s:
                raise ValueError('drop phase timed out')
            if self.issued:
                raise ValueError('release candidate already latched')
            if gate_reason(obs, self.cfg) in ('invalid_observation', 'vision_stale', 'low_confidence', 'height_out_of_range'):
                raise ValueError(gate_reason(obs, self.cfg))
            action = self.policy.act(obs)
            out['candidate_velocity_enu_m_s'] = list(alignment_command(obs, self.cfg))
            reason = gate_reason(obs, self.cfg)
            out['candidate_release'] = bool(action and not reason and snapshot['stage'] == 'RELEASE')
            out['reason'] = reason or ('candidate_latched' if out['candidate_release'] else 'waiting')
            if out['candidate_release']:
                self.issued = True
        except (KeyError, ValueError, TypeError, OverflowError) as exc:
            out['reason'] = str(exc)
        return out
