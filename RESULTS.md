# Evaluation Campaign Results

This document records the experiment run matrix, per-campaign hardware specifications, battery-model assessment, and reproduction instructions for the figures/tables generated under Step 5b.

---

## 1. Run Matrix

The experiments are driven by a centralized config matrix in [campaign_matrix.json](file:///home/px4_sitl/sim_paper/campaign_matrix.json) which defines the exact conditions. It covers 35 unique simulation runs (N = 6, 12, 24; R = 3 or 5 seeds; 25s duration per run).

| Campaign | Label | N | Seeds | Configuration Overrides / Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Scalability** | - | 6, 12, 24 | `101`, `102`, `103` | Tests resource usage and framework overhead up to N = 24. |
| **Stochastic** | Base | 6 | `42`, `43`, `44`, `45`, `46` | Five seeds to capture nominal fleet performance distributions. |
| **Thermal Sensitivity** | low_density | 6 | `42`, `43`, `44` | `num_thermals: 4`, `w_peak_max_mps: 3.0` |
| | high_density | 6 | `42`, `43`, `44` | `num_thermals: 16`, `w_peak_max_mps: 6.0` |
| **Event Sensitivity** | low_load | 6 | `42`, `43`, `44` | `event_rate_hz: 0.02` |
| | high_load | 6 | `42`, `43`, `44` | `event_rate_hz: 0.12` |
| **Baselines & Ablations** | baseline_coverage | 6 | `42`, `43`, `44` | `enable_soaring: false`, `enable_event_investigation: false` |
| | ablation_soaring_only | 6 | `42`, `43`, `44` | `enable_event_investigation: false` |
| | ablation_event_only | 6 | `42`, `43`, `44` | `enable_soaring: false` |

---

## 2. Machine State Specification

To avoid framework-metric contamination (RESOURCE-METRIC ISOLATION), all runs were executed on an otherwise-idle system.

* **CPU**: 13th Gen Intel(R) Core(TM) i9-13900H (20 virtual cores)
* **Memory**: 62 GiB RAM
* **OS**: Linux 6.8.0-111-generic #111~22.04.1-Ubuntu x86_64
* **Real-Time Factor (RTF) Profile**:
  * **N = 6**: ~0.74 (Real-time slowdown factor to prevent estimator divergence and ensure CPU stability).
  * **N = 12**: ~0.62.
  * **N = 24**: ~0.66 (The system scales efficiently to N = 24 with no process failures, but starts showing latency spikes).
* **ROS 2 Message Latency (mean ± 95% CI)**:
  * **N = 6**: 6.01 ± 3.98 ms
  * **N = 12**: 5.35 ± 0.84 ms
  * **N = 24**: 44.38 ± 9.41 ms (Saturates around N = 24 due to high DDS network traffic and CPU virtualization stress, causing average latencies to jump 8x).

---

## 3. Battery-Model Assessment & L/D Validation

The battery estimation is validated against the physical power-required relations under the dynamic fixed-wing aerodynamic equation:
$$P_{aero} = D \cdot V = \left( C_{D0} + k \cdot C_L^2 \right) \cdot q \cdot S \cdot V$$

The collector logs and reports both the bookkeeping baseline (constant L/D = 12) and the dynamically integrated JSBSim L/D values.

### L/D Comparisons Across Configurations

From [baseline_comparison.csv](file:///home/px4_sitl/sim_paper/results/tables/baseline_comparison.csv):

* **Baseline (Coverage-Path Only)**:
  * Bookkeeping Eagle L/D: **12.0**
  * Dynamic JSBSim L/D: **8.56**
* **Ablation (Soaring Only)**:
  * Bookkeeping Eagle L/D: **12.0**
  * Dynamic JSBSim L/D: **8.35**
* **Ablation (Events Only)**:
  * Bookkeeping Eagle L/D: **12.0**
  * Dynamic JSBSim L/D: **10.03**
* **Full Soar & Seek Framework**:
  * Bookkeeping Eagle L/D: **12.0**
  * Dynamic JSBSim L/D: **8.71**

*Conclusion*: The constant Eagle L/D model (12.0) significantly overestimates aerodynamic efficiency compared to the actual physics-based JSBSim model (which ranges between ~8.3 and ~10.0 during fixed-wing turns and glides).

---

## 4. Figure & Table Regeneration

All figures and tables are generated automatically from the raw campaign logs (located under `results/`). No manual editing of data was performed.

### Instructions to Regenerate Figures & Tables
To verify reproducibility and regenerate all deliverables, execute:
```bash
# Delete existing outputs for validation
rm -rf results/figures/* results/tables/*

# Run the aggregator and plotting script
python3 generate_plots.py
```

### Outputs Generated
1. **Scalability**:
   * Table: `results/tables/scalability.csv`
   * Figure: `results/figures/scalability.png` (Four subplots: CPU, Memory, RTF, and Latency vs Swarm Size $N$)
2. **Stochastic Distributions**:
   * Table: `results/tables/stochastic_summary.csv`
   * Figure: `results/figures/stochastic_distribution.png` (Boxplots showing Final SoC, Thermal Exploitation, and Event Detections over 5 seeds)
3. **Thermal & Event Sensitivity**:
   * Table: `results/tables/thermal_sensitivity.csv` & `results/tables/event_sensitivity.csv`
   * Figure: `results/figures/thermal_sensitivity.png` & `results/figures/event_sensitivity.png`
4. **Baselines Comparison**:
   * Table: `results/tables/baseline_comparison.csv`
   * Figure: `results/figures/baseline_comparison.png` (A stacked bar chart comparing propulsion energy by state for the 4 modes)
