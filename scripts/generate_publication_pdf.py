#!/usr/bin/env python3
"""Generate the publication-ready results PDF (18 sections).

Refuses to run unless scripts/validate_results.py passes ALL mandatory gates.
Tables are rendered reader-friendly (<=7 major columns, units, 2-3 sig figs);
full detail lives in results/csv. Every figure is embedded from its 300-dpi PNG.

Output: results/pdf/soar_seek_simulation_results_publication_ready.pdf
"""
import os
import sys
import glob
import json
import subprocess
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from datetime import datetime

WS = "/home/px4_sitl/sim_paper"
CSV = os.path.join(WS, "results/csv")
FIG = os.path.join(WS, "results/figures")
OUT = os.path.join(WS, "results/pdf/soar_seek_simulation_results_publication_ready.pdf")
DARK = '#202124'


def rd(name):
    p = os.path.join(CSV, name)
    return pd.read_csv(p) if os.path.exists(p) else pd.DataFrame()


def fmt(v):
    if isinstance(v, float):
        if np.isnan(v):
            return "NaN"
        if abs(v) >= 1000:
            return f"{v:,.0f}"
        if abs(v) >= 10:
            return f"{v:.1f}"
        return f"{v:.3g}"
    s = str(v)
    return s if len(s) <= 40 else s[:37] + "..."


def text_page(pdf, title, lines, fontsize=9.5):
    fig, ax = plt.subplots(figsize=(11, 8.5)); ax.axis('off')
    ax.text(0.5, 0.96, title, fontsize=17, fontweight='bold', ha='center', va='top', color=DARK)
    ax.text(0.05, 0.88, "\n".join(lines), fontsize=fontsize, va='top',
            fontfamily='monospace', linespacing=1.5, color=DARK)
    pdf.savefig(fig); plt.close(fig)


def table_page(pdf, title, df, caption="", cols=None, rename=None, fontsize=8, max_rows=26):
    if df.empty:
        text_page(pdf, title, ["(no data available)"]); return
    if cols:
        cols = [c for c in cols if c in df.columns]
        df = df[cols]
    if rename:
        df = df.rename(columns=rename)
    chunks = [df.iloc[i:i + max_rows] for i in range(0, len(df), max_rows)] or [df]
    for ci, chunk in enumerate(chunks):
        fig, ax = plt.subplots(figsize=(11, 8.5)); ax.axis('off')
        t = title if len(chunks) == 1 else f"{title} ({ci+1}/{len(chunks)})"
        ax.set_title(t, fontsize=14, fontweight='bold', pad=22, color=DARK)
        cells = [[fmt(v) for v in row] for row in chunk.values]
        tab = ax.table(cellText=cells, colLabels=list(chunk.columns), cellLoc='center', loc='upper center')
        tab.auto_set_font_size(False); tab.set_fontsize(fontsize); tab.scale(1, 1.5)
        for (r, c), cell in tab.get_celld().items():
            cell.set_edgecolor('#cccccc')
            if r == 0:
                cell.set_facecolor('#2b5c8f'); cell.set_text_props(color='white', fontweight='bold')
        if caption and ci == len(chunks) - 1:
            ax.text(0.5, 0.05, caption, fontsize=8.5, ha='center', va='bottom',
                    transform=ax.transAxes, style='italic', color='#444', wrap=True)
        pdf.savefig(fig); plt.close(fig)


def fig_page(pdf, title, png, caption=""):
    path = os.path.join(FIG, png)
    if not os.path.exists(path):
        text_page(pdf, title, [f"(figure {png} missing)"]); return
    img = plt.imread(path)
    fig = plt.figure(figsize=(11, 8.5))
    fig.suptitle(title, fontsize=14, fontweight='bold', color=DARK, y=0.97)
    ax = fig.add_axes([0.03, 0.08, 0.94, 0.84]); ax.imshow(img); ax.axis('off')
    if caption:
        fig.text(0.5, 0.03, caption, fontsize=8.5, ha='center', style='italic', color='#444', wrap=True)
    pdf.savefig(fig); plt.close(fig)


def smean(summary, metric):
    r = summary[summary['metric'] == metric]
    return float(r['mean'].iloc[0]) if len(r) else float('nan')


def main():
    print("Running mandatory quality gates before publication PDF...")
    rc = subprocess.run([sys.executable, os.path.join(WS, "scripts/validate_results.py")]).returncode
    if rc != 0:
        print("ERROR: quality gates failed — publication PDF NOT generated.")
        return 1

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    stoch = rd("stochastic_runs_per_seed.csv")
    summ = rd("stochastic_runs_summary.csv")
    scal = rd("scalability_overhead.csv")
    runq = rd("run_quality_flags.csv")
    uavq = rd("uav_record_quality_flags.csv")
    gates = rd("quality_gates.csv")

    with PdfPages(OUT) as pdf:
        # Title
        fig = plt.figure(figsize=(11, 8.5)); fig.patch.set_facecolor('#1a237e')
        ax = fig.add_axes([0, 0, 1, 1]); ax.axis('off')
        ax.text(0.5, 0.64, 'Soar & Seek', fontsize=44, fontweight='bold', color='white', ha='center')
        ax.text(0.5, 0.53, 'Publication-Ready Simulation-Framework Results', fontsize=20, color='#bbdefb', ha='center')
        ax.text(0.5, 0.42, 'Repeatable distributed simulation of coupled fixed-wing flight,\n'
                'thermals, stochastic ground events, FOV sensing, propulsion energy,\n'
                'and multi-UAV execution (PX4 SITL + JSBSim + MAVSDK + ROS 2)', fontsize=12.5,
                color='#90caf9', ha='center')
        ax.text(0.5, 0.28, 'All values from real SITL logs. No synthetic/fallback data.\n'
                'Energy is propulsion-only. The framework is not claimed optimal.', fontsize=11,
                color='#64b5f6', ha='center')
        ax.text(0.5, 0.12, f'Generated {datetime.now().strftime("%Y-%m-%d %H:%M")}', fontsize=10,
                color='#90caf9', ha='center')
        pdf.savefig(fig); plt.close(fig)

        # 1. Experiment setup
        text_page(pdf, "1. Experiment Setup", [
            "Stack: PX4 SITL (lockstep, real time) + JSBSim fixed-wing FDM per vehicle +",
            "Micro-XRCE-DDS + ROS 2 Humble env nodes (thermal field, ground events, FOV",
            "sensing, propulsion-energy estimator, FSM autonomy via MAVSDK) + collector.",
            "",
            "Scenario: 6-UAV partitioned coverage route (23.3 km total), patrol 150 m AGL,",
            "thermals to 500 m, FSM thermalling ceiling 400 m. Battery scaled (trigger FSM",
            "energy states in-horizon); energy/power are physical (propulsion-only).",
            "",
            "Horizon: 600 s campaign runs, 900 s nominal case study. Duration metrics are",
            "SHORT-HORIZON FRAMEWORK EXECUTION, not vehicle endurance.",
            "",
            "Metric definitions: results/csv/metric_definitions.csv.",
            "Energy-model & scaled-vs-physical battery: results/csv/energy_model_parameters.csv.",
            "Per-UAV IDs/ports/keys/namespaces: results/csv/uav_instance_mapping.csv.",
        ])

        # 2. Experiment matrix
        n_valid = len(runq[runq.get('exclusion_scope', pd.Series()) == 'none']) if not runq.empty else 0
        text_page(pdf, "2. Experiment Matrix", [
            "Campaign            Runs  N        Horizon  Seeds        Purpose",
            "-" * 78,
            "Nominal case study   1    6        900 s    42           End-to-end demo",
            "Repeated stochastic  20   6        600 s    42-61        Distributional stats",
            "Reduced baselines    30   6        600 s    42-46        6 configs x 5 seeds",
            "Thermal sensitivity  10   6        600 s    42-46        low/high (+nominal)",
            "Event sensitivity    10   6        600 s    42-46        low/high (+nominal)",
            "Scalability          9    6/12/24  120 s    101-103      Resource overhead",
            "",
            f"Validated runs: {n_valid}.  Seeds: master->thermal RNG; +100->event RNG.",
            "Route, UAV model and framework parameters fixed across seeds.",
        ])

        # 3. Framework feature comparison
        table_page(pdf, "3. Framework Feature Comparison", rd("framework_feature_comparison.csv"),
                   caption="Verified capabilities (no unfair speed benchmark vs other simulators).",
                   fontsize=8.5)

        # 4. Framework verification
        ver = rd("framework_verification.csv")
        table_page(pdf, "4. Framework Verification", ver,
                   cols=["check_name", "passed", "failed_count", "total_count"],
                   rename={"check_name": "check", "failed_count": "failures", "total_count": "samples"},
                   caption="Data-driven checks over all runs. Quantitative detail: "
                           "px4_jsbsim_consistency / ros2_isolation_checks / mavsdk_routing_checks CSVs.")

        # 5. Coverage comparison
        table_page(pdf, "5. Coverage-Path Comparison", rd("coverage_path_comparison.csv"),
                   cols=["strategy", "total_path_length_km", "mean_per_uav_km", "fov_coverage_pct",
                         "uncovered_area_km2", "workload_imbalance"],
                   rename={"total_path_length_km": "total km", "mean_per_uav_km": "mean/UAV km",
                           "fov_coverage_pct": "FOV cov %", "uncovered_area_km2": "uncov km2",
                           "workload_imbalance": "imbalance"},
                   caption="FOV-based coverage. Selected route is shortest but not highest coverage "
                           "— reasonable, not claimed optimal.")
        fig_page(pdf, "5b. Coverage-Path Comparison", "coverage_path_comparison.png")

        # 6. Planned route
        fig_page(pdf, "6. Planned 23.3 km Coverage Route", "evaluation_region_path.png",
                 caption="Six-UAV partition; per-UAV segments sum to 23.3 km (validated).")

        # 7. Executed path
        fig_page(pdf, "7. Executed Path Overlay", "executed_path_overlay.png",
                 caption="Real logged trajectories: thermalling segments (entry/exit), HP-event "
                         "loiters, glide-return and landing segments over the planned route.")

        # 8. Behaviour zoom
        fig_page(pdf, "8. Thermal & Event Behaviour (zoom)", "path_behaviour_zoom.png",
                 caption="Left: a thermal-exploitation spiral inside an active thermal. "
                         "Right: an HP-event investigation loiter at the event location.")
        fig_page(pdf, "8b. Representative UAV Timeline", "representative_uav_timeline.png")

        # 9. Reduced-framework baselines
        table_page(pdf, "9. Reduced-Framework Baselines", rd("reduced_framework_baselines.csv"),
                   cols=["configuration", "final_soc_pct_mean", "propulsion_energy_wh_mean",
                         "hp_investigated_pct_mean", "thermalling_time_s_mean", "process_failures_mean"],
                   rename={"configuration": "config", "final_soc_pct_mean": "SOC %",
                           "propulsion_energy_wh_mean": "energy Wh", "hp_investigated_pct_mean": "HP inv %",
                           "thermalling_time_s_mean": "therm s", "process_failures_mean": "proc fail"},
                   caption="Component ablations (5 matched seeds). Paired per-seed effects: "
                           "reduced_framework_paired_effects.csv.")
        fig_page(pdf, "9b. Reduced-Framework Baselines", "reduced_framework_baselines.png")

        # 10. Energy assessment
        table_page(pdf, "10. Propulsion-Energy: Battery-Model Comparison", rd("battery_model_comparison.csv"),
                   cols=["model", "final_soc_pct_mean", "mean_propulsion_power_w", "soc_rmse_vs_online_pct"],
                   rename={"final_soc_pct_mean": "final SOC %", "mean_propulsion_power_w": "mean P (W)",
                           "soc_rmse_vs_online_pct": "SOC RMSE %"},
                   caption="Propulsion-only. Self-RMSE = NaN (not a validation); external real-flight "
                           "reference unavailable (NaN).")
        fig_page(pdf, "10b. Propulsion-Energy Breakdown (no avionics)", "energy_budget_breakdown_no_avionics.png")
        fig_page(pdf, "10c. Battery-Model Comparison", "battery_model_comparison.png")
        fig_page(pdf, "10d. SOC Time Series by Mode", "soc_time_series_by_mode.png")

        # 11. Thermal sensitivity
        table_page(pdf, "11. Thermal-Field Sensitivity", rd("thermal_sensitivity.csv"),
                   cols=["level", "thermal_encounters_per_uav_mean", "thermalling_time_s_mean",
                         "thermal_alt_gain_m_mean", "exploitation_saving_wh_mean", "final_soc_pct_mean"],
                   rename={"thermal_encounters_per_uav_mean": "enc/UAV", "thermalling_time_s_mean": "therm s",
                           "thermal_alt_gain_m_mean": "alt gain m", "exploitation_saving_wh_mean": "saving Wh",
                           "final_soc_pct_mean": "SOC %"},
                   caption="5 matched seeds per level; route/events/autonomy fixed.")
        fig_page(pdf, "11b. Thermal-Field Sensitivity", "thermal_sensitivity.png")
        fig_page(pdf, "11c. Thermal Altitude/Energy Trace", "thermal_altitude_energy_trace.png")

        # 12. Event sensitivity
        table_page(pdf, "12. Ground-Event Sensitivity", rd("event_sensitivity.csv"),
                   cols=["level", "total_hp_events_mean", "hp_detected_pct_mean", "hp_investigated_pct_mean",
                         "hp_unresolved_pct_mean", "mean_hp_investigation_latency_s_mean"],
                   rename={"total_hp_events_mean": "HP events", "hp_detected_pct_mean": "det %",
                           "hp_investigated_pct_mean": "inv %", "hp_unresolved_pct_mean": "unres %",
                           "mean_hp_investigation_latency_s_mean": "latency s"},
                   caption="Counts and percentages (not Hz). Full lifecycle: event_lifecycle_records.csv.")
        fig_page(pdf, "12b. Ground-Event Outcomes", "event_sensitivity_outcomes.png")
        fig_page(pdf, "12c. Ground-Event Latency", "event_sensitivity_latency.png")

        # 13. Repeated stochastic
        table_page(pdf, "13. Repeated Stochastic Results (R=%d)" % len(stoch), summ,
                   cols=["metric", "mean", "std", "median", "min", "max", "ci95_halfwidth"],
                   rename={"ci95_halfwidth": "ci95"},
                   caption="Mean/std/median/min/max/95%CI across seeds. Per-seed: "
                           "stochastic_runs_per_seed.csv.", fontsize=7, max_rows=30)
        fig_page(pdf, "13b. Repeated Stochastic Summary", "repeated_stochastic_summary.png")

        # 14. FSM transition stability
        table_page(pdf, "14. FSM Transition Stability", rd("fsm_transition_statistics.csv"),
                   cols=["condition", "transitions_per_uav_per_min_mean", "mean_state_dwell_s",
                         "min_state_dwell_s", "sub_2s_dwell_pct", "rejected_transitions_total"],
                   rename={"transitions_per_uav_per_min_mean": "trans/min", "mean_state_dwell_s": "mean dwell s",
                           "min_state_dwell_s": "min dwell s", "sub_2s_dwell_pct": "sub-2s %",
                           "rejected_transitions_total": "rejected"},
                   caption="Hysteresis + dwell/cooldown + re-investigation cooldown. Sub-2 s chatter "
                           "eliminated (was 27.8%).")
        fig_page(pdf, "14b. FSM Transition Timeline", "fsm_transition_timeline.png")

        # 15. Scalability
        table_page(pdf, "15. Scalability & Resource Overhead", scal,
                   cols=["fleet_size_n", "armed_uavs_mean", "arming_success_pct", "cpu_pct_mean",
                         "rtf_mean", "ros2_latency_ms_mean", "process_failure_count_mean"],
                   rename={"fleet_size_n": "N", "armed_uavs_mean": "armed", "arming_success_pct": "arm %",
                           "cpu_pct_mean": "CPU %", "rtf_mean": "RTF", "ros2_latency_ms_mean": "lat ms",
                           "process_failure_count_mean": "proc fail"},
                   caption="Denominator = REQUESTED fleet size (unarmed UAVs not excluded). "
                           "Dropped-msg rate uninstrumented (NaN).")
        fig_page(pdf, "15b. Scalability & Resource Overhead", "scalability_overhead.png")

        # 16. Exclusions & quality
        n_run_excl = int(runq.get('excluded', pd.Series([], dtype=bool)).sum()) if not runq.empty else 0
        n_uav_excl = len(uavq[uavq.get('uav_id', pd.Series()) != ""]) if not uavq.empty else 0
        npass = int(gates['passed'].sum()) if not gates.empty else 0
        text_page(pdf, "16. Exclusions & Quality Summary", [
            f"Quality gates: {npass}/{len(gates)} passed (all mandatory gates must pass to",
            "produce this PDF). Full report: results/quality/quality_gate_report.txt.",
            "",
            f"Excluded experiment runs: {n_run_excl}",
            f"Excluded individual UAV records: {n_uav_excl}",
            "  (run-level vs UAV-record exclusions are tracked in separate CSVs and",
            "   never conflated: run_quality_flags.csv, uav_record_quality_flags.csv)",
            "",
            "A UAV-record exclusion = one vehicle that failed to produce valid telemetry",
            "in one run; the run is retained for its remaining UAVs. Scalability denominators",
            "still count all requested UAVs.",
        ])
        table_page(pdf, "16b. Quality-Gate Report", gates,
                   cols=["gate", "name", "passed", "detail"], fontsize=7, max_rows=24)

        # 17. Limitations
        text_page(pdf, "17. Limitations", [
            "- Short-horizon execution with a scaled battery: duration figures are framework",
            "  execution time, not vehicle endurance.",
            "- Energy is propulsion-only; avionics/payload/comms/compute excluded.",
            "- No instrumented real-flight energy log -> external battery reference is NaN.",
            "- ROS 2 per-message drop rate not instrumented -> reported as NaN.",
            "- PX4<->JSBSim consistency uses kinematic self-consistency RMSE (no separate FDM",
            "  ground-truth channel logged).",
            "- Thermal/event models are parametric (Allen updraft, Poisson events), not",
            "  reanalysis-driven.",
            "- The framework is evaluated for correctness/repeatability/sensitivity/scalability;",
            "  NO optimal planning or coordination is claimed.",
        ])

        # 18. Reproducibility
        nominal = sorted(glob.glob(os.path.join(WS, "logs/raw/nominal/N_*_seed_*")))
        commits = []
        if nominal:
            mm = json.load(open(os.path.join(nominal[0], "manifest.json")))
            for k, v in mm.get('git_commit_hashes', {}).items():
                commits.append(f"  {k:<16} {v}")
        text_page(pdf, "18. Reproducibility Instructions", [
            "1. cd ros2_ws && colcon build --packages-select soarer_msgs soarer_env",
            "2. python3 scripts/run_all_campaigns.py        # all SITL campaigns",
            "3. python3 scripts/generate_reference_csvs.py  # static reference CSVs",
            "4. python3 scripts/run_coverage_comparison.py  # FOV coverage comparison",
            "5. python3 scripts/verify_framework.py         # verification + split CSVs",
            "6. python3 scripts/aggregate_results.py        # all result tables",
            "7. python3 scripts/plot_results.py             # all PNG+PDF figures",
            "8. python3 scripts/generate_publication_pdf.py # gates + this PDF",
            "",
            "Seeds: configs/seeds.yaml. Scenario: configs/scenario_nominal.yaml.",
            "Scalability port fix: PX4 ROMFS px4-rc.mavlink offboard remote = 14640+instance.",
            "",
            "Framework commit hashes:",
            *commits,
            "",
            "Every results table maps to its source runs in results/quality/provenance.json.",
        ])

    print(f"Publication PDF generated: {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
