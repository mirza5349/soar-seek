# Metrics, Instrumentation, & Reproducibility Harness (Step 5a)

This document describes the structured schema of collected metrics, the directory layout of experiment runs, and the procedure for reproducing runs byte-for-byte at the metrics level.

---

## 1. Metrics Schema

The `metrics_collector_node` collects metrics across two main categories: Mission Metrics (per-UAV and fleet-wide) and Framework Metrics (system-level performance).

### A. Mission Metrics (`metrics_summary.json` & `mission_metrics.csv`)

| Metric Name | JSON Key / CSV Column | Units | Source Topic | Formula / Derivation |
|---|---|---|---|---|
| **Endurance** | `endurance_s` | seconds | `/soarer/telemetry/px4_{i}` | Elapsed time between first and last telemetry message |
| **Final SoC** | `final_soc_pct` | % | `/soarer/battery/px4_{i}` | Latest battery state-of-charge percentage |
| **Propulsion Energy by Mode** | `propulsion_energy_wh` | Wh | `/soarer/battery/px4_{i}`, `/soarer/fsm/px4_{i}` | Integrated `power_draw_w * dt` grouped by the active FSM state |
| **Thermal Encounters** | `thermal_encounters` | count | `/soarer/fsm/px4_{i}` | Number of transitions into the `THERMAL_EXPLOITATION` state |
| **Total Thermalling Time** | `total_thermalling_time_s` | seconds | `/soarer/fsm/px4_{i}` | Cumulative duration spent in `THERMAL_EXPLOITATION` |
| **Detected-Event Rate** | `detected_event_rate_hz` | Hz | `/soarer/fov/px4_{i}` | Total FOV camera hit count divided by endurance |
| **High-Priority Investigation Rate** | `high_priority_investigation_rate_hz` | Hz | `/soarer/fsm/px4_{i}` | Total count of transitions into `EVENT_INVESTIGATION` divided by endurance |
| **HP Expired Count** | `high_priority_expired_count` | count | `/soarer/events` | High-priority events that reached expiry while actively being investigated |
| **HP Unresolved Count** | `high_priority_unresolved_count` | count | `/soarer/events` | High-priority events that reached expiry without being investigated |
| **Mean Investigation Latency** | `mean_investigation_latency_s` | seconds | `/soarer/fov/px4_{i}`, `/soarer/fsm/px4_{i}` | Average time from first detection in footprint to transition to `EVENT_INVESTIGATION` |
| **Landing Success** | `landing_success` | boolean | `/soarer/fsm/px4_{i}`, `/soarer/telemetry/px4_{i}` | `True` if FSM state was `LANDING` and telemetry registers zero velocity |
| **State-Transition Correctness** | `state_transition_correctness` | boolean | `/soarer/fsm/px4_{i}` | `True` if FSM transitions follow valid state sequences |
| **Reported L/D** | `reported_ld` | ratio | `/soarer/telemetry/px4_{i}` | L/D selected by config: dynamic `V / v_sink` (JSBSIM) or constant `12` (EAGLE) |
| **Raw L/D Eagle** | `raw_ld_eagle` | ratio | — | Constant `12.0` bookkeeping value |
| **Raw L/D JSBSim** | `raw_ld_jsbsim` | ratio | `/soarer/telemetry/px4_{i}` | Dynamically resolved aerodynamic L/D from drag and velocity telemetry |

### B. Framework Metrics (`framework_metrics.csv`)

| Metric Name | CSV Column | Units | Source | Description |
|---|---|---|---|---|
| **CPU Percent** | `cpu_percent` | % | System / `psutil` | System-wide CPU utilization |
| **Memory Percent** | `mem_percent` | % | System / `psutil` | System-wide memory utilization |
| **Real-Time Factor** | `rtf` | ratio | `/soarer/telemetry/px4_{i}` | Elapsed simulation time divided by elapsed real wall-clock time |
| **ROS 2 Latency** | `avg_ros_latency_ms` | ms | Node loop clock | Average latency from message stamp to receipt for topics |
| **Log Size** | `log_size_kb` | KB | File system | Sum of sizes of active PX4, JSBSim, and env node logs |

---

## 2. Run Directory Layout

Every run orchestrated by `run_experiment.py` generates a self-contained, self-describing directory:

```
run_N_<fleet_size>_seed_<seed>_<timestamp>/
├── config.yaml              # The exact YAML configuration used for the run
├── manifest.json            # Run metadata, commits, seed, and summary execution metrics
├── metrics_summary.json     # Detailed mission-level metrics for all UAVs
├── mission_metrics.csv      # Flat CSV mapping of all mission metrics
├── framework_metrics.csv    # Chronological CSV sampling of CPU, memory, RTF, and log sizes
└── raw_logs/                # Collected raw stdout logs for diagnostics
    ├── soarer_env.log       # Environmental nodes log
    ├── MicroXRCEAgent.log   # DDS Agent log
    ├── uav_1_px4_1.log      # UAV 1 PX4 stdout log
    └── uav_1_jsbsim_bridge_1.log
```

---

## 3. Reproducibility Procedure

The seeded run orchestrator uses the master seed to deterministically configure the stochastic parameters (thermal fields generator and Poisson ground events) and records the framework versions (git commit hashes).

To repeat or audit any run from its manifest:
1. Ssh into a shell configured with the same codebase.
2. Read the parameters from the run's `manifest.json`.
3. Execute the experiment runner with the same arguments:
   ```bash
   ./run_experiment.py --seed <seed> --n <N> --duration <duration> --ld-flag <ld_flag> --output-dir <new_dir>
   ```
4. Differentiate the resulting `metrics_summary.json` from the audited run's file. The results will align identically within thread scheduler tolerances.
