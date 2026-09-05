"""Comparable baselines and tabular Q-learning release policy."""
from bisect import bisect_right
import json
import math
from pathlib import Path
from .observation import current_error, gate_reason, nominal_miss


class Baseline:
    def __init__(self, name, config):
        if name not in ('hover', 'ballistic', 'random'):
            raise ValueError(name)
        self.name, self.cfg = name, config
        self.unknown = 0

    def act(self, obs, rng=None):
        if gate_reason(obs, self.cfg):
            return 0
        if self.name == 'random':
            if rng is None:
                raise ValueError('random policy requires an explicit RNG')
            return int(rng.random() < 0.15)
        if self.name == 'hover':
            return int(math.hypot(*current_error(obs)) < obs.radius * 0.6
                       and math.hypot(obs.vx, obs.vy) < 0.15 and obs.stable >= 0.5)
        return int(math.hypot(*nominal_miss(obs, self.cfg)) < obs.radius * 0.6)


def state_key(obs, config):
    miss = math.hypot(*nominal_miss(obs, config)) / obs.radius
    speed = math.hypot(obs.vx, obs.vy)
    ex, ey = current_error(obs)
    closing = int(ex * obs.vx + ey * obs.vy > 0)
    bins = (bisect_right((0.2, 0.4, 0.6, 0.8, 1.0, 1.4, 2.0, 3.0), miss),
            bisect_right((0.05, 0.1, 0.15, 0.2, 0.25), speed), closing,
            bisect_right((0.3, 0.6, 0.85), obs.elapsed / config.horizon_s),
            int(not gate_reason(obs, config)))
    return ','.join(map(str, bins))


class QPolicy:
    def __init__(self, config, table=None):
        self.cfg = config
        self.table = table if table is not None else {}
        self.unknown = 0

    def act(self, obs, rng=None, epsilon=0.0):
        if gate_reason(obs, self.cfg):
            return 0
        key = state_key(obs, self.cfg)
        if rng is not None and rng.random() < epsilon:
            return rng.randrange(2)
        if key not in self.table:
            self.unknown += 1
        values = self.table.get(key, (0.0, 0.0))
        return int(values[1] > values[0])  # unseen/tied states wait

    def update(self, obs, action, reward, next_obs, terminal, alpha=0.15, gamma=0.98):
        values = self.table.setdefault(state_key(obs, self.cfg), [0.0, 0.0])
        future = self.table.get(state_key(next_obs, self.cfg), (0.0, 0.0))
        next_value = future[0] if gate_reason(next_obs, self.cfg) else max(future)
        target = reward if terminal else reward + gamma * next_value
        values[action] += alpha * (target - values[action])

    def save(self, path, metadata=None):
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps({'format': 'cuadc-q-v1', 'config': self.cfg.to_dict(),
                                 'metadata': metadata or {}, 'table': self.table}, indent=2), encoding='utf-8')

    @classmethod
    def load(cls, path, config=None):
        from .config import Config
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if data.get('format') != 'cuadc-q-v1':
            raise ValueError('unsupported model format')
        for k, values in data['table'].items():
            if not isinstance(k, str) or len(values) != 2 or not all(math.isfinite(v) for v in values):
                raise ValueError('invalid Q table')
        return cls(config or Config(**data['config']).validate(), data['table'])
