#!/usr/bin/env python3
"""Reduced-framework baseline configurations (framework-component comparison,
not algorithmic benchmarks).

Configurations:
  full_framework      - reuses logs/raw/stochastic seeds 42-44 (identical
                        config + seeds; documented in the results PDF)
  no_thermal          - soaring disabled
  no_event_response   - event investigation disabled
  non_energy_aware_fsm- SOC thresholds disabled (no thermal search trigger,
                        no reserve landing)
  simplified_battery  - constant-power discharge model instead of the
                        aerodynamic propulsion estimator
  coverage_only       - soaring AND event response disabled
"""
import os
import sys
import yaml
sys.path.insert(0, os.path.dirname(__file__))
from campaign_lib import WORKSPACE, run_one, already_done, restore_nominal_config

CONFIGS = {
    "no_thermal": {
        "fsm_node": {"ros__parameters": {"enable_soaring": False}}},
    "no_event_response": {
        "fsm_node": {"ros__parameters": {"enable_event_investigation": False}}},
    "non_energy_aware_fsm": {
        "fsm_node": {"ros__parameters": {"search_soc": -100.0, "reserve_soc": -100.0}}},
    "simplified_battery": {
        "battery_estimator_node": {"ros__parameters": {"battery_model": "constant"}}},
    "coverage_only": {
        "fsm_node": {"ros__parameters": {"enable_soaring": False,
                                         "enable_event_investigation": False}}},
}


def main():
    with open(os.path.join(WORKSPACE, "configs/seeds.yaml")) as f:
        seeds = yaml.safe_load(f)["baseline_seeds"]

    failures = []
    for name, overrides in CONFIGS.items():
        for seed in seeds:
            out_dir = os.path.join(WORKSPACE, f"logs/raw/baselines/{name}/N_6_seed_{seed}")
            if already_done(out_dir):
                print(f"Skipping {name} seed {seed} (already complete).")
                continue
            if not run_one(out_dir, seed=seed, n=6, duration=600, override_dict=overrides):
                failures.append((name, seed))

    restore_nominal_config()
    if failures:
        print(f"FAILED runs: {failures}")
        sys.exit(1)
    print("Baseline campaigns finished.")


if __name__ == "__main__":
    main()
