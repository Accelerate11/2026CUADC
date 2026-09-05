"""Run the complete dependency-free path using this same Python interpreter."""
import subprocess
import sys
import _bootstrap


def main():
    root = _bootstrap.ROOT
    commands = [
        ['-m', 'unittest', 'discover', '-s', 'tests', '-v'],
        ['scripts/ballistic_sweep.py'],
        ['scripts/train_q.py', '--episodes', '6000'],
        ['scripts/evaluate.py', '--episodes', '300'],
        ['scripts/evaluate.py', '--config', 'configs/stress.json', '--seed', '81000000', '--out', 'outputs/stress'],
        ['scripts/demo.py', '--policy', 'q', '--out', 'outputs/demo_q.html'],
        ['scripts/demo.py', '--policy', 'hover', '--out', 'outputs/demo_hover.html'],
        ['integration/replay_telemetry.py', 'examples/telemetry.jsonl'],
    ]
    for command in commands:
        print('\nRunning: ' + ' '.join(command), flush=True)
        subprocess.run([sys.executable, *command], cwd=root, check=True)
    print('\nDone. Open outputs/demo_q.html and outputs/evaluation.json.')


if __name__ == '__main__':
    main()
