"""Standalone release-delay/velocity experiment, without training or packages."""
import argparse
import _bootstrap
from drop_rl.physics import ballistic_landing
from drop_rl.experiments import write_csv


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--out', default='outputs/ballistic_sweep.csv')
    args = p.parse_args()
    rows = []
    for h in (0.7, 1.2, 1.7):
        for v in (0.0, 0.1, 0.25, 0.5):
            for delay in (0.0, 0.1, 0.2):
                # Constant aircraft velocity during this isolated sweep's release delay.
                landing, fall_s = ballistic_landing((v*delay, 0), (v, 0), h)
                rows.append({'height_above_rim_m': h, 'vx_m_s': v, 'delay_s': delay,
                             'fall_s': fall_s, 'drift_x_m': landing[0]})
    write_csv(args.out, rows)
    print(f'{len(rows)} cases -> {args.out}')


if __name__ == '__main__':
    main()
