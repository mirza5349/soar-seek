#!/usr/bin/env python3
"""Nominal six-UAV end-to-end mission (case study).

Longer horizon (300 s simulated) so the full energy-aware FSM cycle —
patrol, FOV event detection, HP investigation, thermal exploitation,
glide return and reserve landing — executes within one run.
"""
import argparse
import os
import sys
sys.path.insert(0, os.path.dirname(__file__))
from campaign_lib import WORKSPACE, run_one, restore_nominal_config


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--duration", type=int, default=900,
                    help="Simulated mission horizon in seconds")
    ap.add_argument("--force", action="store_true")
    args = ap.parse_args()

    # Larger battery for the long-horizon case study so the reserve landing
    # triggers near the end of the 600 s window instead of mid-run.
    overrides = {"battery_estimator_node": {"ros__parameters": {"battery_capacity_wh": 5.5}}}
    out_dir = os.path.join(WORKSPACE, f"logs/raw/nominal/N_6_seed_{args.seed}")
    ok = run_one(out_dir, seed=args.seed, n=6, duration=args.duration,
                 override_dict=overrides)
    restore_nominal_config()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
