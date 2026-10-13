#!/usr/bin/env python3
"""打包完整提案目录，排除编译产物、运行日志、缓存；不修改源码。"""
import argparse
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[1]
EXCLUDED = {"__pycache__", "build", "install", "log", "logs", ".git", ".pytest_cache"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT.with_suffix(".zip"))
    args = parser.parse_args()
    output = args.out.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        count = 0
        for path in sorted(ROOT.rglob("*")):
            relative = path.relative_to(ROOT)
            if (not path.is_file() or path.resolve() == output
                    or any(p in EXCLUDED or p.startswith(("build_", "install_", "log_"))
                           for p in relative.parts[:-1])
                    or path.suffix in {".pyc", ".zip"} or path.name == ".flight_serial.lock"):
                continue
            archive.write(path, (Path(ROOT.name) / relative).as_posix())
            count += 1
    print(f"Packaged {count} files: {output} ({output.stat().st_size:,} bytes)")


if __name__ == "__main__":
    main()
