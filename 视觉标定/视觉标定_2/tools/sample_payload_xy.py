#!/usr/bin/env python3
"""Sample one detected bucket and report median/MAD/residual in formal FLU."""
import argparse
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--payload", type=int, choices=(1, 2), required=True)
    ap.add_argument("--true-x", type=float, required=True)
    ap.add_argument("--true-y", type=float, required=True)
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--topic", default="/perception/drop_buckets_body")
    ap.add_argument("--min-samples", type=int, default=5)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node(f"payload_{args.payload}_xy_sampler")
    samples = []

    def cb(msg: PoseArray):
        if len(msg.poses) != 1:
            return
        p = msg.poses[0].position
        values = [float(p.x), float(p.y), float(p.z)]
        if all(np.isfinite(values)):
            samples.append(values)

    sub = node.create_subscription(PoseArray, args.topic, cb, 10)
    deadline = time.monotonic() + max(0.5, args.duration)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if len(samples) < args.min_samples:
        print(f"[ERROR] valid samples={len(samples)} < {args.min_samples}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)

    a = np.asarray(samples, dtype=np.float64)
    med = np.median(a, axis=0)
    mad = np.median(np.abs(a - med), axis=0)
    dx = args.true_x - med[0]
    dy = args.true_y - med[1]

    print(f"samples = {len(a)}")
    print(f"median FLU = x={med[0]:+.4f}, y={med[1]:+.4f}, z={med[2]:+.4f} m")
    print(f"MAD        = x={mad[0]:.4f}, y={mad[1]:.4f}, z={mad[2]:.4f} m")
    print()
    print(f"P{args.payload} reference FLU = x={args.true_x:+.4f}, y={args.true_y:+.4f} m")
    print(f"remaining dx = {dx:+.4f} m")
    print(f"remaining dy = {dy:+.4f} m")
    print(f"pass_5mm = {'YES' if abs(dx) <= 0.005 and abs(dy) <= 0.005 else 'NO'}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
