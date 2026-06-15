#!/usr/bin/env python3
"""Generate the revised results PDF.

Refuses to run unless scripts/validate_results.py passes all 13 quality
gates. Every figure page embeds the PNG generated from raw logs; every
table page renders a results/csv table verbatim.

Output: results/pdf/soar_seek_simulation_results_revised.pdf
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

WORKSPACE = "/home/px4_sitl/sim_paper"
CSV_DIR = os.path.join(WORKSPACE, "results/csv")
FIG_DIR = os.path.join(WORKSPACE, "results/figures")
OUT_PDF = os.path.join(WORKSPACE, "results/pdf/soar_seek_simulation_results_revised.pdf")

DARK = '#202124'


def read_csv(name):
    return pd.read_csv(os.path.join(CSV_DIR, name))


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
    return s if len(s) <= 38 else s[:35] + "..."


def title_text_page(pdf, title, lines, fontsize=9.5, family='monospace'):
    fig, ax = plt.subplots(figsize=(11, 8.5))
    ax.axis('off')
    ax.text(0.5, 0.96, title, fontsize=18, fontweight='bold', ha='center',
            va='top', color=DARK)
    ax.text(0.05, 0.88, "\n".join(lines), fontsize=fontsize, va='top',
            fontfamily=family, linespacing=1.5, color=DARK)
    pdf.savefig(fig)
    plt.close(fig)


def table_page(pdf, title, df, caption="", col_width_scale=1.0, fontsize=7.5,
               max_rows=28):
    chunks = [df.iloc[i:i + max_rows] for i in range(0, len(df), max_rows)] or [df]
    for ci, chunk in enumerate(chunks):
        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.axis('off')
        t = title if len(chunks) == 1 else f"{title} ({ci + 1}/{len(chunks)})"
        ax.set_title(t, fontsize=15, fontweight='bold', pad=24, color=DARK)
        cells = [[fmt(v) for v in row] for row in chunk.values]
        tab = ax.table(cellText=cells, colLabels=list(chunk.columns),
                       cellLoc='center', loc='upper center')
        tab.auto_set_font_size(False)
        tab.set_fontsize(fontsize)
        tab.scale(col_width_scale, 1.45)
        for (r, c), cell in tab.get_celld().items():
            cell.set_edgecolor('#cccccc')
            if r == 0:
                cell.set_facecolor('#2b5c8f')
                cell.set_text_props(color='white', fontweight='bold', fontsize=fontsize - 0.5)
        if caption and ci == len(chunks) - 1:
            ax.text(0.5, 0.04, caption, fontsize=8.5, ha='center', va='bottom',
                    transform=ax.transAxes, style='italic', color='#444444', wrap=True)
        pdf.savefig(fig)
        plt.close(fig)


def figure_page(pdf, title, png_name, caption=""):
    path = os.path.join(FIG_DIR, png_name)
    img = plt.imread(path)
    fig = plt.figure(figsize=(11, 8.5))
    fig.suptitle(title, fontsize=15, fontweight='bold', color=DARK, y=0.97)
    ax = fig.add_axes([0.03, 0.08, 0.94, 0.84])
    ax.imshow(img)
    ax.axis('off')
    if caption:
        fig.text(0.5, 0.03, caption, fontsize=8.5, ha='center', style='italic',
                 color='#444444', wrap=True)
    pdf.savefig(fig)
    plt.close(fig)


def main():
    # ---- Quality gates must pass first ----------------------------------
    print("Running quality gates before PDF generation...")
    rc = subprocess.run([sys.executable,
                         os.path.join(WORKSPACE, "scripts/validate_results.py")]).returncode
    if rc != 0:
        print("ERROR: quality gates failed — PDF not generated.")
        return 1

    os.makedirs(os.path.dirname(OUT_PDF), exist_ok=True)

    stoch = read_csv("stochastic_runs_per_seed.csv")
    summary = read_csv("stochastic_runs_summary.csv").set_index("metric")
    nominal = read_csv("nominal_summary.csv")
    quality = read_csv("quality_gates.csv")
    flags = read_csv("run_quality_flags.csv")
    scal = read_csv("scalability_overhead.csv")

    def s_mean(metric):
        try:
            return summary.loc[metric, 'mean']
        except KeyError:
            return float('nan')

    n_runs_total = len(flags[flags['issue'] == 'none'])
    excluded = flags[flags['excluded'] == True]  # noqa: E712

    with PdfPages(OUT_PDF) as pdf:
        # ---- 1. Title ----------------------------------------------------
        fig = plt.figure(figsize=(11, 8.5))
        fig.patch.set_facecolor('#1a237e')
        ax = fig.add_axes([0, 0, 1, 1])
        ax.axis('off')
        ax.text(0.5, 0.66, 'Soar & Seek', fontsize=46, fontweight='bold',
                color='white', ha='center')
        ax.text(0.5, 0.54, 'Revised Simulation-Framework Evaluation Results',
                fontsize=21, color='#bbdefb', ha='center')
        ax.text(0.5, 0.42, 'A repeatable distributed simulation framework for coupled\n'
                'fixed-wing flight, thermals, ground events, sensing,\n'
                'propulsion-energy state and multi-UAV execution',
                fontsize=13, color='#90caf9', ha='center')
        ax.text(0.5, 0.28, 'All values are computed from raw PX4-SITL/JSBSim/ROS 2 logs.\n'
                'No synthetic or fallback data. Energy figures are propulsion-only.',
                fontsize=11, color='#64b5f6', ha='center')
        ax.text(0.5, 0.12, f'Generated: {datetime.now().strftime("%Y-%m-%d %H:%M")}',
                fontsize=10, color='#90caf9', ha='center')
        pdf.savefig(fig)
        plt.close(fig)

        # ---- 2. Executive summary ----------------------------------------
        fleet_row = nominal[nominal['uav_id'] == 'FLEET'].iloc[0] if (nominal['uav_id'] == 'FLEET').any() else None
        lines = [
            "Scope. This report evaluates Soar & Seek as a REPEATABLE SIMULATION FRAMEWORK:",
            "framework verification, component-level (reduced-framework) comparisons,",
            "sensitivity studies, repeated stochastic experiments, trajectory and thermal",
            "behaviour, event processing, propulsion-energy assessment, and scalability.",
            "It does NOT claim an optimal planning or coordination algorithm.",
            "",
            f"Experiment volume: {n_runs_total} validated SITL runs "
            f"(plus {len(excluded)} excluded with documented reasons).",
            "",
            "Key repeated-run statistics (R = %d stochastic seeds, N = 6 UAVs):" % len(stoch),
            f"  execution duration ......... {s_mean('execution_duration_s'):.1f} s  "
            f"(sd {summary.loc['execution_duration_s','std']:.1f})",
            f"  final SOC (propulsion) ..... {s_mean('final_soc_pct'):.1f} %",
            f"  fleet thermal encounters ... {s_mean('thermal_encounters'):.1f}",
            f"  fleet thermalling time ..... {s_mean('thermalling_time_s'):.1f} s",
            f"  events detected ............ {s_mean('all_detected_pct'):.1f} %",
            f"  HP events investigated ..... {s_mean('hp_investigated_pct'):.1f} %",
            f"  mean HP latency ............ {s_mean('mean_hp_investigation_latency_s'):.1f} s",
            f"  process failures ........... {s_mean('process_failures'):.2f} per run",
            "",
            "Horizon honesty: campaign runs execute a 600 s simulated horizon (900 s for",
            "the end-to-end case study) at realistic vertical scale (patrol 150 m,",
            "thermalling ceiling 400 m, 2-4 min thermal climbs) with a scaled 2.5 Wh",
            "battery so the full energy-aware FSM cycle completes in-window. Results are",
            "SHORT-HORIZON FRAMEWORK EXECUTION metrics, not vehicle endurance claims.",
            "",
            "Energy wording: all energy values are PROPULSION-ONLY (no avionics, payload,",
            "communication or compute power). Thermal-exploitation benefits are reported",
            "as propulsion-energy savings relative to powered cruise, never as generated",
            "electrical energy.",
        ]
        title_text_page(pdf, "Executive Summary", lines)

        # ---- 3. Experiment matrix ----------------------------------------
        lines = [
            "Campaign                  Runs  N    Sim horizon  Seeds          Purpose",
            "-" * 95,
            "Nominal case study        1     6    900 s        42             End-to-end mission demo",
            "Repeated stochastic       20    6    600 s        42-61          Distributional statistics",
            "Reduced-framework         15    6    600 s        42-44          5 ablation configs",
            "  (+ full_framework reuses stochastic seeds 42-44: identical config & seeds)",
            "Thermal sensitivity       10    6    600 s        42-46          low/high (nominal = stochastic 42-46)",
            "Event sensitivity         10    6    600 s        42-46          low/high (nominal = stochastic 42-46)",
            "Scalability               9     6/12/24  120 s    101-103        Resource overhead",
            "",
            "Stack: PX4 SITL (lockstep) + JSBSim fixed-wing FDM + Micro-XRCE-DDS +",
            "ROS 2 Humble environment nodes (thermal field, ground events, FOV sensing,",
            "propulsion-energy estimator, FSM autonomy via MAVSDK) + metrics collector.",
            "",
            "Seeding: master seed drives the thermal field; ground events use seed+100",
            "(arrival times, locations, priority allocation). Route, UAV model and",
            "framework parameters are fixed across seeds.",
        ]
        title_text_page(pdf, "Experiment Matrix", lines, fontsize=8.5)

        # ---- 4. Framework verification ------------------------------------
        table_page(pdf, "Framework Verification", read_csv("framework_verification.csv"),
                   caption="Data-driven checks over all nominal + stochastic runs; "
                           "failed_count/total_count are per-sample counts from raw logs.")

        # ---- 5. Coverage-path comparison ----------------------------------
        table_page(pdf, "Coverage-Path Comparison", read_csv("coverage_path_comparison.csv"),
                   caption="Geometric comparison over the same operational region (convex hull "
                           "of the selected route). The selected partitioned route trades some "
                           "coverage gap for ~35% shorter per-UAV segments. This demonstrates the "
                           "route is reasonable; no coverage-planning optimality is claimed.")
        figure_page(pdf, "Evaluation Region and Planned Route", "evaluation_region_path.png")

        # ---- 6. Executed path overlay --------------------------------------
        figure_page(pdf, "Executed Path Overlay (real trajectories)",
                    "executed_path_overlay.png",
                    caption="Executed UAV trajectories from SITL logs over the planned route: "
                            "thermal footprints, circular thermalling segments with entry/exit "
                            "markers, HP-event investigation loiters, glide-return and landing "
                            "segments. FSM-state segments are cross-checked against the logged "
                            "transition times.")
        figure_page(pdf, "Representative UAV Timeline", "representative_uav_timeline.png",
                    caption="FSM state, altitude + updraft, SOC and propulsion power over time "
                            "for the most active UAV of the case-study run.")

        # ---- 7. Reduced-framework baselines --------------------------------
        table_page(pdf, "Reduced-Framework Baselines",
                   read_csv("reduced_framework_baselines.csv"), fontsize=5.6,
                   caption="Framework-component comparison (not algorithmic benchmarks). "
                           "no_event_response preserves energy but fails the surveillance "
                           "objective; no_thermal removes thermalling benefits; "
                           "non_energy_aware_fsm removes the energy-safety net; "
                           "simplified_battery changes SOC interpretation.")
        figure_page(pdf, "Reduced-Framework Baselines", "reduced_framework_baselines.png")

        # ---- 8. Energy ------------------------------------------------------
        table_page(pdf, "Propulsion-Energy Budget by FSM Mode",
                   read_csv("energy_budget_by_mode.csv"),
                   caption="Propulsion-only energy (no avionics). The SAVING row is the "
                           "estimated propulsion-energy saving relative to powered cruise during "
                           "motor-off thermalling — not generated electrical energy.")
        figure_page(pdf, "Propulsion-Energy Breakdown (No Avionics)",
                    "energy_budget_breakdown_no_avionics.png")
        figure_page(pdf, "SOC Time Series by FSM Mode", "soc_time_series_by_mode.png")
        table_page(pdf, "Battery-Model Comparison", read_csv("battery_model_comparison.csv"),
                   caption="Online aerodynamic propulsion-only estimator vs constant-power "
                           "discharge re-evaluated on the same logged timelines vs PX4's own "
                           "simulated battery estimate (reference).")

        # ---- 9. Thermal sensitivity ----------------------------------------
        table_page(pdf, "Thermal-Field Sensitivity", read_csv("thermal_sensitivity.csv"),
                   fontsize=5.6,
                   caption="Low/nominal/high thermal availability, 5 seeds per condition; "
                           "route, events and autonomy fixed.")
        figure_page(pdf, "Thermal-Field Sensitivity", "thermal_sensitivity.png")
        figure_page(pdf, "Thermal Altitude/Energy Trace", "thermal_altitude_energy_trace.png",
                    caption="Altitude gain with flat propulsion-energy curve during the logged "
                            "THERMAL_EXPLOITATION segment confirms coupled thermal-flight-energy "
                            "behaviour.")

        # ---- 10. Event sensitivity ------------------------------------------
        table_page(pdf, "Ground-Event Sensitivity", read_csv("event_sensitivity.csv"),
                   fontsize=5.2,
                   caption="Counts and percentages (not rates in Hz). Low/nominal/high event "
                           "load, 5 seeds per condition.")
        figure_page(pdf, "Ground-Event Sensitivity — Outcomes", "event_sensitivity_outcomes.png")
        figure_page(pdf, "Ground-Event Sensitivity — Latency", "event_sensitivity_latency.png")

        # ---- 11. Stochastic runs ---------------------------------------------
        table_page(pdf, "Repeated Stochastic Runs — Summary (mean / std)",
                   read_csv("stochastic_runs_summary.csv"),
                   caption=f"R = {len(stoch)} seeds; per-seed values in "
                           "stochastic_runs_per_seed.csv. All durations validated non-negative.")
        figure_page(pdf, "Repeated Stochastic Runs", "repeated_stochastic_summary.png")

        # ---- 12. Scalability --------------------------------------------------
        table_page(pdf, "Scalability and Resource Overhead", scal, fontsize=6.0)
        rtf_min = scal['rtf_mean'].min()
        rtf_note = ("Mean RTF stays at or above 1.0 for all fleet sizes."
                    if rtf_min >= 1.0 else
                    f"Mean RTF drops to {rtf_min:.2f} at the largest fleet size: execution is "
                    "hardware-limited and slower than real time; we do not claim real-time "
                    "operation at that scale.")
        arm_note = ""
        if 'uav_arming_success_pct_mean' in scal.columns:
            arm = dict(zip(scal['fleet_size_n'], scal['uav_arming_success_pct_mean']))
            arm_note = (" Resource overhead (CPU, memory) scales as expected and process "
                        f"spawning succeeds at all sizes, but vehicle arming success degrades "
                        f"with fleet size on this 20-core host: "
                        + ", ".join(f"N={int(k)}: {v:.0f}%" for k, v in sorted(arm.items()))
                        + ". UAVs that never armed are excluded from mission aggregates and "
                        "documented in run_quality_flags.csv — this is a reported hardware/"
                        "scale limitation, not a silent failure.")
        figure_page(pdf, "Scalability and Resource Overhead", "scalability_overhead.png",
                    caption=rtf_note + arm_note)

        # ---- 13. Quality gates -------------------------------------------------
        table_page(pdf, "Quality-Gate Report", quality, fontsize=7.0,
                   caption="All gates must pass before this PDF can be generated.")
        excl_df = flags[flags['excluded'] == True]  # noqa: E712
        if len(excl_df):
            table_page(pdf, "Excluded Runs (documented)", excl_df, fontsize=7.0)

        # ---- 14. Reproducibility -------------------------------------------------
        nominal_run = sorted(glob.glob(os.path.join(WORKSPACE, "logs/raw/nominal/N_*_seed_*")))
        commit_lines = []
        if nominal_run:
            man = json.load(open(os.path.join(nominal_run[0], "manifest.json")))
            for k, v in man.get('git_commit_hashes', {}).items():
                commit_lines.append(f"  {k:<16} {v}")
        hw = open(os.path.join(WORKSPACE, "hardware.txt")).read().strip().splitlines() \
            if os.path.exists(os.path.join(WORKSPACE, "hardware.txt")) else []
        lines = [
            "Reproduction steps:",
            "  1. colcon build (ros2_ws), source install/setup.bash",
            "  2. python3 scripts/run_all_campaigns.py        # all SITL campaigns (~6 h)",
            "  3. python3 scripts/run_coverage_comparison.py  # geometric route comparison",
            "  4. python3 scripts/verify_framework.py         # verification suite",
            "  5. python3 scripts/aggregate_results.py        # results/csv tables",
            "  6. python3 scripts/plot_results.py             # PNG + PDF figures",
            "  7. python3 scripts/generate_results_pdf.py     # gates + this PDF",
            "",
            "Seeds: configs/seeds.yaml. Scenario: configs/scenario_nominal.yaml.",
            "Sweep overrides: configs/thermal_{low,nominal,high}.yaml,",
            "                 configs/events_{low,nominal,high}.yaml.",
            "",
            "Framework commit hashes (from run manifests):",
            *commit_lines,
            "",
            "Hardware:",
            *[f"  {l}" for l in hw],
            "",
            "Data layout: logs/raw/<campaign>/<run>/ holds per-run traces",
            "(uav_trace_*.csv), FSM transitions, thermal/event/detection ledgers,",
            "framework_metrics.csv, manifest.json and the exact config.yaml used.",
            "results/quality/provenance.json maps every table to its source runs.",
        ]
        title_text_page(pdf, "Reproducibility Notes", lines, fontsize=9)

    print(f"PDF generated: {OUT_PDF}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
