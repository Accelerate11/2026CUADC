import csv
import json
import math
import random
import statistics
from pathlib import Path
from .env import DropEnv


def run_episode(policy, config, seed, record=False):
    env = DropEnv(config, record=record)
    obs = env.reset(seed)
    rng = random.Random(seed + 900000001)
    total = 0.0
    while not env.done:
        action = policy.act(obs, rng=rng)
        if isinstance(action, tuple):
            release, residual = action
        else:
            release, residual = action, (0.0, 0.0)
        obs, reward, terminated, truncated, info = env.step(release, residual)
        total += reward
    return {**info, 'return': total, 'episode_s': env.t}, env


def percentile(values, q):
    if not values:
        return None
    v = sorted(values)
    x = (len(v) - 1) * q
    lo, hi = math.floor(x), math.ceil(x)
    return v[lo] + (v[hi] - v[lo]) * (x - lo)


def summarize(rows):
    n = len(rows)
    hits = sum(r['hit'] for r in rows)
    errors = [r['error_m'] for r in rows if r['error_m'] is not None]
    release_times = [r['decision_s'] for r in rows if r['decision_s'] is not None]
    # Wilson interval; includes timeouts/out-of-bounds in the hit-rate denominator.
    p, z = hits / n, 1.959963984540054
    den = 1 + z*z/n
    center = (p + z*z/(2*n)) / den
    half = z * math.sqrt(p*(1-p)/n + z*z/(4*n*n)) / den
    return {'episodes': n, 'hits': hits, 'hit_rate': p,
            'hit_rate_ci95': [center-half, center+half],
            'release_rate': len(errors)/n,
            'mean_error_m_released_only': statistics.mean(errors) if errors else None,
            'p90_error_m_released_only': percentile(errors, 0.9),
            'mean_decision_s_released_only': statistics.mean(release_times) if release_times else None,
            'mean_episode_s': statistics.mean(r['episode_s'] for r in rows),
            'mean_return': statistics.mean(r['return'] for r in rows),
            'timeouts': sum(r['outcome'] == 'timeout' for r in rows),
            'out_of_bounds': sum(r['outcome'] == 'out_of_bounds' for r in rows)}


def write_json(path, data):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')


def write_csv(path, rows):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    with p.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
