#!/usr/bin/env python3
"""Thermal-field sensitivity sweep: low / nominal / high settings.

Varies thermal count (effective spawn rate), radius, strength and lifetime
while keeping route, UAV model, event process and autonomy logic fixed.
The 'nominal' condition reuses logs/raw/stochastic seeds 42-46 (identical
configuration and seeds).
"""
import os
import sys
import yaml
sys.path.insert(0, os.path.dirname(__file__))
from campaign_lib import WORKSPACE, run_one, already_done, restore_nominal_config

LEVELS = {
    "low": os.path.join(WORKSPACE, "configs/thermal_low.yaml"),
    "high": os.path.join(WORKSPACE, "configs/thermal_high.yaml"),
}


def main():
    with open(os.path.join(WORKSPACE, "configs/seeds.yaml")) as f:
        seeds = yaml.safe_load(f)["sensitivity_seeds"]

    failures = []
    for level, override in LEVELS.items():
        for seed in seeds:
            out_dir = os.path.join(
                WORKSPACE, f"logs/raw/thermal_sensitivity/{level}/N_6_seed_{seed}")
            if already_done(out_dir):
                print(f"Skipping thermal {level} seed {seed} (already complete).")
                continue
            if not run_one(out_dir, seed=seed, n=6, duration=600,
                           override_files=[override]):
                failures.append((level, seed))

    restore_nominal_config()
    if failures:
        print(f"FAILED runs: {failures}")
        sys.exit(1)
    print("Thermal sensitivity campaign finished.")


if __name__ == "__main__":
    main()
