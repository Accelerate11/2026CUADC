#!/usr/bin/env python3
"""Validate and summarize the append-only JSONL written by safety_monitor."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Optional, Sequence


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    path = args.log.expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"log does not exist: {path}")
    event_types: Counter[str] = Counter()
    states: Counter[str] = Counter()
    faults = []
    previous_sequence = -1
    records = 0
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as error:
            raise SystemExit(f"invalid JSON at line {line_number}: {error}") from error
        sequence = int(record.get("sequence", -1))
        if sequence <= previous_sequence:
            raise SystemExit(f"non-monotonic sequence at line {line_number}")
        previous_sequence = sequence
        records += 1
        event_type = str(record.get("type", "unknown"))
        event_types[event_type] += 1
        if event_type == "mission_status":
            states[str(record.get("state", "unknown"))] += 1
        if bool(record.get("critical", False)):
            faults.append(record)
    print(f"log={path}")
    print(f"records={records}")
    print("event_types=" + json.dumps(event_types, sort_keys=True))
    print("mission_states=" + json.dumps(states, sort_keys=True))
    print(f"critical_faults={len(faults)}")
    return 1 if faults else 0


if __name__ == "__main__":
    raise SystemExit(main())
