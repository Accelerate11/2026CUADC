"""Replay locally recorded JSONL snapshots and print shadow decisions."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drop_rl.config import load_config
from drop_rl.integration import ShadowSupervisor
from drop_rl.policies import Baseline, QPolicy


def main():
    p = argparse.ArgumentParser()
    p.add_argument('input', help='one snapshot per JSON line')
    p.add_argument('--config')
    p.add_argument('--model')
    args = p.parse_args()
    cfg = load_config(args.config)
    policy = QPolicy.load(args.model, cfg) if args.model else Baseline('ballistic', cfg)
    supervisor = ShadowSupervisor(policy, cfg)
    for line in Path(args.input).read_text(encoding='utf-8-sig').splitlines():
        if line.strip():
            print(json.dumps(supervisor.decide(json.loads(line)), ensure_ascii=False))


if __name__ == '__main__':
    main()
