#!/usr/bin/env python3
"""Repeated stochastic runs: nominal full framework across 20 seeds.

Each master seed drives thermal generation (thermal_field seed) and the
ground-event process (seed + 100: activation times, locations, priority
allocation). Route, UAV model, fleet size and framework parameters stay
fixed. Seeds 42-46 double as the 'nominal' condition of the sensitivity
sweeps, and seeds 42-44 as the 'full_framework' baseline configuration.
"""
import os
import sys
import yaml
sys.path.insert(0, os.path.dirname(__file__))
from campaign_lib import WORKSPACE, run_one, already_done, restore_nominal_config


def main():
    with open(os.path.join(WORKSPACE, "configs/seeds.yaml")) as f:
        seeds = yaml.safe_load(f)["stochastic_seeds"]

    failures = []
    for seed in seeds:
        out_dir = os.path.join(WORKSPACE, f"logs/raw/stochastic/N_6_seed_{seed}")
        if already_done(out_dir):
            print(f"Skipping seed {seed} (already complete).")
            continue
        if not run_one(out_dir, seed=seed, n=6, duration=600):
            failures.append(seed)

    restore_nominal_config()
    if failures:
        print(f"FAILED seeds: {failures}")
        sys.exit(1)
    print("Stochastic campaign finished.")


if __name__ == "__main__":
    main()
