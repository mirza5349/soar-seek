#!/usr/bin/env python3
import os
import sys
import time
import argparse
import subprocess
import shutil
import json
import yaml
import psutil

def get_git_commit(path):
    try:
        res = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=path,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=True
        )
        return res.stdout.strip()
    except Exception:
        return "unknown"

def main():
    parser = argparse.ArgumentParser(description="Soarer Swarm Seeded Run Orchestrator")
    parser.add_argument("--seed", type=int, default=42, help="Master PRNG seed")
    parser.add_argument("--n", type=int, default=2, help="Swarm size N")
    parser.add_argument("--duration", type=int, default=60, help="Run duration in seconds")
    parser.add_argument("--ld-flag", type=str, choices=["EAGLE", "JSBSIM"], default="JSBSIM", help="Reported L/D selection")
    parser.add_argument("--output-dir", type=str, default=None, help="Target run directory path")
    parser.add_argument("--no-metrics", action="store_true", help="Disable metrics collector node")
    args = parser.parse_args()

    workspace_dir = "/home/px4_sitl/sim_paper"
    px4_dir = os.path.join(workspace_dir, "PX4-Autopilot")
    px4_msgs_dir = os.path.join(workspace_dir, "ros2_ws/src/px4_msgs")
    jsbsim_src_dir = os.path.join(workspace_dir, "jsbsim_src")

    # Determine output directory
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    if args.output_dir is None:
        run_dir = os.path.join(workspace_dir, f"run_N_{args.n}_seed_{args.seed}_{timestamp}")
    else:
        run_dir = os.path.abspath(args.output_dir)

    print("=================================================================")
    print(" Running Seeded Experiment Orchestrator")
    print(f"   Swarm Size N:      {args.n}")
    print(f"   Master Seed:       {args.seed}")
    print(f"   Duration:          {args.duration} s")
    print(f"   L/D Report Flag:   {args.ld_flag}")
    print(f"   Output Directory:  {run_dir}")
    print("=================================================================")

    # Step 1: Clear existing simulation processes
    print("Step 1/7: Running swarm teardown for safety...")
    subprocess.run([os.path.join(workspace_dir, "swarm_teardown.sh")], check=True)

    # Step 2: Capture git commits
    print("Step 2/7: Capturing framework commit hashes...")
    px4_commit = get_git_commit(px4_dir)
    msgs_commit = get_git_commit(px4_msgs_dir)
    jsbsim_commit = get_git_commit(jsbsim_src_dir)
    print(f"   PX4-Autopilot:        {px4_commit}")
    print(f"   px4_msgs workspace:   {msgs_commit}")
    print(f"   jsbsim_src:           {jsbsim_commit}")

    # Step 3: Read and update config.yaml
    print("Step 3/7: Injecting PRNG seeds and experiment parameters into config...")
    config_path = os.path.join(workspace_dir, "ros2_ws/src/soarer_env/config/config.yaml")
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # Inject seeds
    config['thermal_field_node']['ros__parameters']['seed'] = args.seed
    config['thermal_field_node']['ros__parameters']['num_vehicles'] = args.n
    config['ground_event_node']['ros__parameters']['seed'] = args.seed + 100
    
    # Configure collector
    config['metrics_collector_node']['ros__parameters']['reported_value_flag'] = args.ld_flag
    config['metrics_collector_node']['ros__parameters']['num_vehicles'] = args.n
    config['metrics_collector_node']['ros__parameters']['output_dir'] = run_dir

    # Write updated config back
    with open(config_path, 'w') as f:
        yaml.safe_dump(config, f, default_flow_style=False)

    # Step 4: Launch simulation
    print("Step 4/7: Launching swarm simulation...")
    # Export ROS_DOMAIN_ID and LAUNCH_METRICS
    os.environ["ROS_DOMAIN_ID"] = "10"
    os.environ["LAUNCH_METRICS"] = "false" if args.no_metrics else "true"
    
    # Run launcher as a subprocess and wait for completion of boot
    launch_res = subprocess.run([os.path.join(workspace_dir, "swarm_launcher.sh"), str(args.n)], check=True)
    
    # Lockstep simulation runs at ~0.9x real-time.
    # Therefore, args.duration simulated seconds requires approx (args.duration / 0.85) wall seconds.
    # We add a 20s margin for boot and takeoff settling.
    wall_duration = (float(args.duration) / 0.85) + 20.0

    # Wait for execution duration and landing completion
    print(f"Step 5/7: Simulation running. Monitoring landing states (wall timeout: {wall_duration:.1f}s)...")
    pid_file = "/tmp/soarer_swarm.pids"
    pids = []
    if os.path.exists(pid_file):
        with open(pid_file, 'r') as f:
            pids = [int(line.strip()) for line in f if line.strip().isdigit()]

    process_failures = 0
    failed_pids = set()

    # MAVLink connections to monitor arm/disarm transitions
    from pymavlink import mavutil
    connections = {}
    for i in range(1, args.n + 1):
        port = 14550 + i
        try:
            connections[i] = mavutil.mavlink_connection(f"udpin:localhost:{port}")
        except Exception as e:
            print(f"  Warning: failed to establish MAVLink connection to UAV {i} on port {port}: {e}")

    has_armed = {i: False for i in range(1, args.n + 1)}
    has_landed = {i: False for i in range(1, args.n + 1)}

    # MAVLink heartbeat timeout accounting: a timeout is a gap > 3 s between
    # heartbeats from a vehicle that has already been heard from.
    last_heartbeat = {i: None for i in range(1, args.n + 1)}
    mavlink_timeouts = {i: 0 for i in range(1, args.n + 1)}
    in_timeout = {i: False for i in range(1, args.n + 1)}

    start_time = time.time()

    while time.time() - start_time < wall_duration:
        # Check tracked PIDs for unexpected exit
        for pid in pids:
            if pid not in failed_pids and not psutil.pid_exists(pid):
                process_failures += 1
                failed_pids.add(pid)

        # Check arming states
        all_landed = True
        now = time.time()
        for i, conn in connections.items():
            if has_landed[i]:
                continue

            all_landed = False
            # Drain all pending messages (heartbeats share the queue with the
            # full GCS telemetry stream; a single recv per cycle falls behind
            # and produces false timeout counts).
            try:
                msg = None
                while True:
                    m = conn.recv_match(type='HEARTBEAT', blocking=False)
                    if m is None:
                        break
                    msg = m
                if msg:
                    last_heartbeat[i] = now
                    in_timeout[i] = False
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if armed:
                        if not has_armed[i]:
                            has_armed[i] = True
                            print(f"  [UAV {i}] Armed and taking off...")
                    else:
                        if has_armed[i]:
                            has_landed[i] = True
                            print(f"  [UAV {i}] Disarmed / Landed successfully!")
                elif last_heartbeat[i] is not None and not in_timeout[i] \
                        and now - last_heartbeat[i] > 3.0:
                    mavlink_timeouts[i] += 1
                    in_timeout[i] = True
                    print(f"  [UAV {i}] MAVLink heartbeat timeout (> 3 s without heartbeat).")
            except Exception:
                pass

        if all_landed and len(has_landed) > 0 and time.time() - start_time > 15.0:
            print("All UAVs have successfully landed and disarmed. Ending simulation early!")
            break

        time.sleep(0.5)

    # Step 5a: Gracefully finalize the metrics collector before teardown so
    # the final summary/trace flush completes (teardown SIGKILLs after 2 s).
    print("Step 6/7: Finalizing metrics collector, then tearing down swarm...")
    subprocess.run(["pkill", "-INT", "-f", "metrics_collector_node"], check=False)
    summary_path = os.path.join(run_dir, "metrics_summary.json")
    for _ in range(20):
        time.sleep(0.5)
        if os.path.exists(summary_path) and time.time() - os.path.getmtime(summary_path) < 5.0:
            break

    subprocess.run([os.path.join(workspace_dir, "swarm_teardown.sh")], check=True)

    # Step 6: Harvest logs and metrics
    print("Step 7/7: Packaging self-contained run directory...")
    os.makedirs(run_dir, exist_ok=True)
    
    # Copy active config file used for the run
    shutil.copy(config_path, os.path.join(run_dir, "config.yaml"))

    # Harvest log files
    raw_logs_dir = os.path.join(run_dir, "raw_logs")
    os.makedirs(raw_logs_dir, exist_ok=True)

    # Move ROS 2 environmental log
    env_log = os.path.join(workspace_dir, "ros2_ws/soarer_env.log")
    if os.path.exists(env_log):
        shutil.copy(env_log, os.path.join(raw_logs_dir, "soarer_env.log"))

    # Move MicroXRCEAgent log
    agent_log = os.path.join(workspace_dir, "MicroXRCEAgent.log")
    if os.path.exists(agent_log):
        shutil.copy(agent_log, os.path.join(raw_logs_dir, "MicroXRCEAgent.log"))

    # Move PX4 and JSBSim logs from rootfs
    build_dir = os.path.join(px4_dir, "build/px4_sitl_default")
    for i in range(1, args.n + 1):
        rootfs = os.path.join(build_dir, f"rootfs_{i}")
        if os.path.exists(rootfs):
            for file in os.listdir(rootfs):
                if file.endswith(".log"):
                    shutil.copy(os.path.join(rootfs, file), os.path.join(raw_logs_dir, f"uav_{i}_{file}"))

    # Log sizes calculation
    total_log_bytes = 0
    for root, _, files in os.walk(raw_logs_dir):
        for file in files:
            total_log_bytes += os.path.getsize(os.path.join(root, file))
    total_log_kb = total_log_bytes / 1024.0

    # Write Manifest file
    manifest = {
        "manifest_format_version": "1.0",
        "timestamp": timestamp,
        "parameters": {
            "seed": args.seed,
            "fleet_size_n": args.n,
            "duration_s": args.duration,
            "reported_ld_flag": args.ld_flag
        },
        "git_commit_hashes": {
            "PX4-Autopilot": px4_commit,
            "px4_msgs": msgs_commit,
            "jsbsim_src": jsbsim_commit
        },
        "execution_summary": {
            "process_failure_count": process_failures,
            "total_log_size_kb": total_log_kb,
            "mavlink_timeout_count": int(sum(mavlink_timeouts.values())),
            "mavlink_timeouts_per_uav": {str(k): int(v) for k, v in mavlink_timeouts.items()},
            "uavs_armed": int(sum(1 for v in has_armed.values() if v)),
            "uavs_landed_disarmed": int(sum(1 for v in has_landed.values() if v)),
            "wall_duration_s": round(time.time() - start_time, 1)
        }
    }

    manifest_path = os.path.join(run_dir, "manifest.json")
    with open(manifest_path, 'w') as f:
        json.dump(manifest, f, indent=4)

    print("=================================================================")
    print(f" Run successfully completed and packaged!")
    print(f" Run Directory: {run_dir}")
    print(f" Manifest:      {manifest_path}")
    print("=================================================================")

if __name__ == "__main__":
    main()
