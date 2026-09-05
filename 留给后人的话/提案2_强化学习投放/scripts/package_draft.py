"""Make a portable local zip; excludes venv, runtime output and caches."""
import argparse
import zipfile
from pathlib import Path
import _bootstrap


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='../cuadc_drop_rl_草案.zip')
    args = p.parse_args()
    root, destination = _bootstrap.ROOT, Path(args.out).resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    excluded = {'outputs', '__pycache__', '.git', '.pytest_cache'}
    count = 0
    with zipfile.ZipFile(destination, 'w', compression=zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(root.rglob('*')):
            relative = path.relative_to(root)
            if any(part in excluded or part.startswith('.venv') for part in relative.parts):
                continue
            if path.is_file() and path.resolve() != destination and path.suffix not in ('.pyc', '.pyo'):
                archive.write(path, Path(root.name) / relative)
                count += 1
    print(f'{count} files -> {destination} ({destination.stat().st_size:,} bytes)')


if __name__ == '__main__':
    main()
