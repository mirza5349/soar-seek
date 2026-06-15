# Soar & Seek: Evaluation & Results Campaign Runner

This repository contains the evaluation campaigns, configuration files, aircraft
flight dynamics models, launch files, raw logs, processed data, and plotting
scripts for **"Soar & Seek: A Distributed Simulation Framework for Thermal-Aware
UAV Swarms in Surveillance Missions."**

The evaluation demonstrates Soar & Seek as a **repeatable simulation framework**
for coupled fixed-wing flight dynamics, thermals, ground events, sensing,
propulsion-energy state and distributed multi-UAV execution. It does **not**
claim an optimal planning or coordination algorithm.

---

## 1. Project Directory Structure

```text
configs/
  scenario_nominal.yaml        # Canonical scenario (route, thermal field, events, battery)
  thermal_low|nominal|high.yaml# Thermal-field sweep overrides
  events_low|nominal|high.yaml # Ground-event sweep overrides
  seeds.yaml                   # All campaign seed lists
aircraft/                      # JSBSim fixed-wing soarer FDM
launch/                        # Micro-XRCE-DDS / launch scripts
logs/
  raw/<campaign>/<run>/        # Per-run raw outputs (see below)
  processed/                   # Convenience aggregates
results/
  csv/                         # All result tables (one CSV per table)
  figures/                     # All figures (PNG 300 dpi + vector PDF)
  pdf/                         # soar_seek_simulation_results_revised.pdf
  quality/                     # provenance.json, quality_gates.csv
scripts/
  run_nominal.py               # 900 s six-UAV end-to-end case study
  run_stochastic_runs.py       # 20-seed repeated stochastic campaign
  run_baselines.py             # Reduced-framework configurations
  run_thermal_sensitivity.py   # low/high thermal sweeps (nominal = stochastic)
  run_event_sensitivity.py     # low/high event-load sweeps
  run_scalability.py           # N = 6, 12, 24 resource-overhead runs
  run_all_campaigns.py         # Sequential master runner (idempotent, resumable)
  run_coverage_comparison.py   # Geometric coverage-path comparison
  verify_framework.py          # Data-driven verification suite
  aggregate_results.py         # Raw logs -> results/csv (with quality flags)
  plot_results.py              # All figures from real logs/CSVs
  validate_results.py          # 13 quality gates
  generate_results_pdf.py      # Final PDF (refuses to run if gates fail)
  archive_old_results.py       # Archive/delete previous results (asks first)
```

### Per-run raw outputs (`logs/raw/<campaign>/<run>/`)

| File | Content |
| :--- | :--- |
| `uav_trace_<i>.csv` | ~2 Hz position/velocity/altitude (direct from PX4), FSM state, SOC, propulsion power, energy integral, thermal updraft, PX4 reference battery, waypoint progress |
| `fsm_transitions.csv` | Every FSM transition with sim timestamp |
| `thermal_field.csv` | 1 Hz thermal footprint snapshots (position, radius, strength) |
| `events.csv` | Per-event ledger: location, priority, lifetime, first detection, investigation, outcome |
| `detections.csv` | Every FOV detection (timestamped with the position sample used) |
| `framework_metrics.csv` | 1 Hz CPU, memory, RTF (windowed + cumulative), ROS 2 latency, log size |
| `metrics_summary.json` | Per-UAV + fleet metrics (counts and percentages) |
| `manifest.json` | Seeds, commits, process failures, MAVLink timeouts, wall duration |
| `config.yaml` | Exact configuration used for the run |

---

## 2. Campaign Setup & Horizon Honesty

Campaign runs execute a **600 s simulated horizon** (900 s for the nominal
case study) at a realistic vertical scale: patrol at 150 m AGL, thermals up
to 500 m, FSM thermalling ceiling 400 m — so individual thermal climbs last
**2–4 minutes** (~250 m of altitude gain). The battery is programmatically
scaled to **2.5 Wh** (4.0 Wh for the case study) so that the full
energy-aware FSM cycle — patrol, thermal search at 40 % SOC, exploitation,
glide return and 7 % reserve landing — completes inside the horizon at the
~10–25 W propulsion draw of the 1.5 kg aircraft. All duration metrics are
therefore **short-horizon framework execution** results, not vehicle
endurance claims.

All energy values are **propulsion-only** (no avionics, payload, communication
or compute power). Thermal-exploitation benefits are reported as
**propulsion-energy savings relative to powered cruise**, never as generated
electrical energy.

---

## 3. Running the Campaigns

```bash
cd ros2_ws && colcon build --packages-select soarer_msgs soarer_env
source install/setup.bash && cd ..

python3 scripts/run_all_campaigns.py          # all campaigns (~9 h), resumable
```

Individual campaigns can be run with the dedicated `scripts/run_*.py` runners;
each skips runs that already have a `metrics_summary.json`.

Seeding: the master seed drives the thermal field; ground events use
`seed + 100` (arrival times, locations, priority allocation). Route, UAV
model and framework parameters stay fixed across seeds.

---

## 4. Producing the Results Package

```bash
python3 scripts/archive_old_results.py        # archive/delete old results (asks first)
python3 scripts/run_coverage_comparison.py    # geometric route comparison
python3 scripts/verify_framework.py           # results/csv/framework_verification.csv
python3 scripts/aggregate_results.py          # all results/csv tables + run_quality_flags.csv
python3 scripts/plot_results.py               # all PNG+PDF figures
python3 scripts/generate_results_pdf.py       # runs the 13 quality gates, then builds
                                              # results/pdf/soar_seek_simulation_results_revised.pdf
```

Validation rules enforced by the pipeline:

* execution durations must be non-negative — a violation aborts with the run ID;
* per-UAV timestamps are sanitised (out-of-order PX4 samples rejected and counted);
* metrics that cannot be computed are written as `NaN`, never replaced by
  synthetic values;
* `results/quality/provenance.json` maps every table to its source runs;
* the final PDF cannot be generated unless all 13 gates pass and the old
  results PDF has been archived or deleted.
