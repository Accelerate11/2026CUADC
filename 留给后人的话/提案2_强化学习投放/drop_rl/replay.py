"""Self-contained offline HTML replay, no CDN or web server required."""
import json
from pathlib import Path


def export_html(env, path, policy_name):
    payload = {'frames': env.trace, 'result': env.result, 'config': env.cfg.to_dict(),
               'policy': policy_name, 'seed': env.seed}
    data = json.dumps(payload, ensure_ascii=True, allow_nan=False).replace('<', '\\u003c')
    template = (Path(__file__).parent / 'replay_template.html').read_text(encoding='utf-8')
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(template.replace('/*REPLAY_DATA*/null', data), encoding='utf-8')
