"""Freeze only the named reproducible results into the portable examples folder."""
import hashlib
import json
import shutil
from pathlib import Path
import _bootstrap


def main():
    root = _bootstrap.ROOT
    source, target = root / 'outputs', root / 'examples/results'
    target.mkdir(parents=True, exist_ok=True)
    names = ['q_policy.json', 'q_policy.json.training.csv', 'ballistic_sweep.csv',
             'evaluation.json', 'evaluation.csv', 'stress.json', 'stress.csv',
             'bucket15.json', 'bucket15.csv', 'bucket20.json', 'bucket20.csv',
             'demo_q.html', 'demo_q.html.json', 'demo_hover.html', 'demo_hover.html.json',
             'vision_loss.html', 'vision_loss.html.json', 'replay_preview.png',
             'ppo_smoke.zip', 'ppo_smoke.metadata.json', 'ppo_smoke.monitor.csv',
             'ppo_training.log', 'ppo_evaluation.json', 'ppo_evaluation.json.csv']
    manifest, missing = {}, []
    for name in names:
        path = source / name
        if not path.exists():
            missing.append(name)
            continue
        shutil.copy2(path, target / name)
        manifest[name] = {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'bytes': path.stat().st_size}
    (target / 'manifest.json').write_text(json.dumps({'files': manifest, 'not_generated': missing},
                                                    indent=2, ensure_ascii=False), encoding='utf-8')
    print(f'Archived {len(manifest)} files; not generated: {missing}')


if __name__ == '__main__':
    main()
