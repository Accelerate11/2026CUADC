#!/usr/bin/env python3
"""维护者重新生成提案源码来源清单与 SHA256；不运行 ROS 或连接设备。"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED_DIRS = {"build", "install", "log", "logs", "__pycache__", ".git", ".pytest_cache"}
EXCLUDED_FILES = {"SOURCE_MANIFEST.json", "SHA256SUMS", ".flight_serial.lock"}


def source_files():
    for path in sorted(ROOT.rglob("*")):
        rel = path.relative_to(ROOT)
        if (not path.is_file() or any(p in EXCLUDED_DIRS for p in rel.parts)
                or path.name in EXCLUDED_FILES or path.suffix == ".pyc"):
            continue
        yield path, rel.as_posix()


def main():
    baseline = json.loads((ROOT / "BASELINE_SOURCE.json").read_text(encoding="utf-8"))
    files = []
    checksum_lines = []
    for path, rel in source_files():
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        original = baseline["files"].get(rel)
        change = "added" if original is None else (
            "unchanged" if original["sha256"] == digest else "modified")
        files.append({"destination": rel, "change": change,
                      "source_sha256": original["sha256"] if original else None,
                      "packaged_sha256": digest})
        checksum_lines.append(f"{digest}  {rel}")
    manifest = {"version": "guided-stage-proposal-1", "baseline_repository": baseline["repository"],
                "baseline_commit": baseline["commit"], "baseline_path": baseline["repository_prefix"],
                "date": "2026-10-13", "files": files}
    manifest_path = ROOT / "SOURCE_MANIFEST.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    checksum_lines.append(f"{hashlib.sha256(manifest_path.read_bytes()).hexdigest()}  SOURCE_MANIFEST.json")
    (ROOT / "SHA256SUMS").write_text("\n".join(checksum_lines) + "\n", encoding="utf-8")
    counts = {label: sum(f["change"] == label for f in files)
              for label in ("unchanged", "modified", "added")}
    print(json.dumps({"files": len(files), "changes": counts}, ensure_ascii=False))


if __name__ == "__main__":
    main()
