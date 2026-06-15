#!/usr/bin/env python3
"""Master campaign sequencer: runs every experiment group in order and
keeps going even if an individual campaign reports failures (each run
script is idempotent and skips completed runs, so re-running this script
resumes where it stopped)."""
import os
import subprocess
import sys
import time

SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))

CAMPAIGNS = [
    "run_nominal.py",
    "run_stochastic_runs.py",
    "run_baselines.py",
    "run_thermal_sensitivity.py",
    "run_event_sensitivity.py",
    "run_scalability.py",
]


def main():
    t0 = time.time()
    statuses = {}
    for script in CAMPAIGNS:
        path = os.path.join(SCRIPTS_DIR, script)
        print(f"\n##### CAMPAIGN: {script} (t+{(time.time()-t0)/60:.1f} min) #####",
              flush=True)
        res = subprocess.run([sys.executable, path])
        statuses[script] = res.returncode
        print(f"##### {script} finished rc={res.returncode} #####", flush=True)

    print("\n================ CAMPAIGN SUMMARY ================")
    for script, rc in statuses.items():
        print(f"  {script}: {'OK' if rc == 0 else f'FAILURES (rc={rc})'}")
    print(f"Total wall time: {(time.time()-t0)/3600:.2f} h")
    sys.exit(0 if all(rc == 0 for rc in statuses.values()) else 1)


if __name__ == "__main__":
    main()
