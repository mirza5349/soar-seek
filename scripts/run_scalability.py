#!/usr/bin/env python3
"""Scalability and resource-overhead campaign: N = 6, 12, 24 UAVs.

Shorter horizon (120 s simulated) — the purpose is resource/overhead
measurement (CPU, memory, RTF, ROS 2 latency, MAVLink timeouts, process
failures, log size), not mission outcomes.
"""
import os
import sys
import yaml
sys.path.insert(0, os.path.dirname(__file__))
from campaign_lib import WORKSPACE, run_one, already_done, restore_nominal_config

FLEET_SIZES = [6, 12, 24]


def main():
    with open(os.path.join(WORKSPACE, "configs/seeds.yaml")) as f:
        seeds = yaml.safe_load(f)["scalability_seeds"]

    failures = []
    for n in FLEET_SIZES:
        for seed in seeds:
            out_dir = os.path.join(WORKSPACE, f"logs/raw/scalability/N_{n}_seed_{seed}")
            if already_done(out_dir):
                print(f"Skipping N={n} seed {seed} (already complete).")
                continue
            if not run_one(out_dir, seed=seed, n=n, duration=120):
                failures.append((n, seed))

    restore_nominal_config()
    if failures:
        print(f"FAILED runs: {failures}")
        sys.exit(1)
    print("Scalability campaign finished.")


if __name__ == "__main__":
    main()
