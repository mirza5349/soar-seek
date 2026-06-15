#!/usr/bin/env python3
import os
import sys
import json
import yaml
import shutil
import subprocess

def merge_dicts(base, overrides):
    for k, v in overrides.items():
        if isinstance(v, dict) and k in base and isinstance(base[k], dict):
            merge_dicts(base[k], v)
        else:
            base[k] = v
    return base

def main():
    workspace_dir = "/home/px4_sitl/sim_paper"
    matrix_path = os.path.join(workspace_dir, "campaign_matrix.json")
    config_path = os.path.join(workspace_dir, "ros2_ws/src/soarer_env/config/config.yaml")
    backup_config_path = config_path + ".backup"

    if not os.path.exists(matrix_path):
        print(f"Error: Matrix file not found at {matrix_path}")
        sys.exit(1)

    with open(matrix_path, 'r') as f:
        matrix = json.load(f)

    # Backup the original config
    shutil.copy(config_path, backup_config_path)
    print("Backed up default config.yaml.")

    try:
        total_runs = sum(len(c["seeds"]) for c in matrix)
        run_idx = 0
        print(f"Starting Campaign Execution: {len(matrix)} conditions, {total_runs} total runs.")

        for cond in matrix:
            campaign = cond["campaign"]
            N = cond["N"]
            seeds = cond["seeds"]
            duration = cond["duration"]
            ld_flag = cond["ld_flag"]
            overrides = cond.get("overrides", {})
            label = cond.get("label", "")

            # Formulate folder structure
            campaign_dir_name = f"{campaign}_{label}" if label else campaign
            campaign_dir = os.path.join(workspace_dir, "results", campaign_dir_name)

            for seed in seeds:
                run_idx += 1
                run_dir = os.path.join(campaign_dir, f"N_{N}_seed_{seed}")
                print(f"\n[{run_idx}/{total_runs}] Running {campaign_dir_name} | N={N} | Seed={seed} | Duration={duration}s")
                print(f"  Target Run Directory: {run_dir}")

                # 1. Restore backup config to get clean base
                shutil.copy(backup_config_path, config_path)

                # 2. Apply overrides
                if overrides:
                    print(f"  Applying overrides: {json.dumps(overrides)}")
                    with open(config_path, 'r') as f:
                        current_config = yaml.safe_load(f)
                    
                    merged_config = merge_dicts(current_config, overrides)
                    
                    with open(config_path, 'w') as f:
                        yaml.safe_dump(merged_config, f, default_flow_style=False)

                # 3. Call run_experiment.py
                cmd = [
                    sys.executable,
                    os.path.join(workspace_dir, "run_experiment.py"),
                    "--seed", str(seed),
                    "--n", str(N),
                    "--duration", str(duration),
                    "--ld-flag", ld_flag,
                    "--output-dir", run_dir
                ]
                
                # Run the experiment
                res = subprocess.run(cmd)
                if res.returncode != 0:
                    print(f"  Warning: Run failed with exit code {res.returncode}")
                else:
                    print(f"  Completed successfully!")

    except KeyboardInterrupt:
        print("\nExecution interrupted by user.")
    finally:
        # Restore backup config.yaml
        if os.path.exists(backup_config_path):
            shutil.move(backup_config_path, config_path)
            print("Restored original config.yaml.")
        print("Campaign Execution finished.")

if __name__ == "__main__":
    main()
