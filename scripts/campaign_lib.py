#!/usr/bin/env python3
"""Shared helpers for Soar & Seek evaluation campaigns.

Every campaign run:
  1. copies configs/scenario_nominal.yaml to the live ROS 2 config,
  2. merges optional override YAML files / dicts on top,
  3. delegates to run_experiment.py (which injects seeds and output_dir).
"""
import os
import sys
import yaml
import shutil
import subprocess
import time

WORKSPACE = "/home/px4_sitl/sim_paper"
SCENARIO = os.path.join(WORKSPACE, "configs/scenario_nominal.yaml")
LIVE_CONFIG = os.path.join(WORKSPACE, "ros2_ws/src/soarer_env/config/config.yaml")


def merge_dicts(base, overrides):
    for k, v in overrides.items():
        if isinstance(v, dict) and k in base and isinstance(base[k], dict):
            merge_dicts(base[k], v)
        else:
            base[k] = v
    return base


def prepare_config(override_files=None, override_dict=None):
    with open(SCENARIO, 'r') as f:
        cfg = yaml.safe_load(f)
    for path in (override_files or []):
        with open(path, 'r') as f:
            ov = yaml.safe_load(f) or {}
        merge_dicts(cfg, ov)
    if override_dict:
        merge_dicts(cfg, override_dict)
    with open(LIVE_CONFIG, 'w') as f:
        yaml.safe_dump(cfg, f, default_flow_style=False)


def run_one(out_dir, seed, n=6, duration=240, override_files=None,
            override_dict=None, max_attempts=2):
    """Run a single seeded experiment; returns True on success."""
    out_dir = os.path.abspath(out_dir)
    for attempt in range(1, max_attempts + 1):
        prepare_config(override_files, override_dict)
        cmd = [
            sys.executable, os.path.join(WORKSPACE, "run_experiment.py"),
            "--seed", str(seed),
            "--n", str(n),
            "--duration", str(duration),
            "--ld-flag", "JSBSIM",
            "--output-dir", out_dir,
        ]
        print(f"\n=== RUN {out_dir} (seed={seed}, N={n}, dur={duration}s, attempt {attempt}) ===",
              flush=True)
        res = subprocess.run(cmd)
        summary = os.path.join(out_dir, "metrics_summary.json")
        if res.returncode == 0 and os.path.exists(summary):
            return True
        print(f"  RUN FAILED (rc={res.returncode}, summary_exists={os.path.exists(summary)}). "
              f"{'Retrying...' if attempt < max_attempts else 'Giving up.'}", flush=True)
        subprocess.run([os.path.join(WORKSPACE, "swarm_teardown.sh")], check=False)
        time.sleep(5)
    return False


def already_done(out_dir):
    return os.path.exists(os.path.join(out_dir, "metrics_summary.json"))


def restore_nominal_config():
    shutil.copy(SCENARIO, LIVE_CONFIG)
