#!/usr/bin/env python3
import os
import json
import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def compute_ci(data):
    if len(data) < 2:
        return 0.0
    # 95% Confidence Interval half-width using standard t-distribution approximation
    return 1.96 * np.std(data, ddof=1) / np.sqrt(len(data))

def read_json(path):
    if not os.path.exists(path):
        return None
    with open(path, 'r') as f:
        return json.load(f)

def read_csv_cols(path):
    if not os.path.exists(path):
        return None
    try:
        return pd.read_csv(path)
    except Exception:
        return None

def main():
    workspace_dir = "/home/px4_sitl/sim_paper"
    results_dir = os.path.join(workspace_dir, "results")
    tables_dir = os.path.join(results_dir, "tables")
    figures_dir = os.path.join(results_dir, "figures")

    os.makedirs(tables_dir, exist_ok=True)
    os.makedirs(figures_dir, exist_ok=True)

    print("==================================================")
    print(" Running Data Aggregator & Plotter")
    print("==================================================")

    # --------------------------------------------------
    # 1. SCALABILITY CAMPAIGN ANALYSIS
    # --------------------------------------------------
    print("\nProcessing Scalability Campaign...")
    scalability_data = []
    for N in [6, 12, 24]:
        cpus, mems, rtfs, latencies, failures = [], [], [], [], []
        for seed in [101, 102, 103]:
            run_dir = os.path.join(results_dir, "scalability", f"N_{N}_seed_{seed}")
            
            # Read framework metrics
            fw_path = os.path.join(run_dir, "framework_metrics.csv")
            df = read_csv_cols(fw_path)
            if df is not None and not df.empty:
                cpus.append(df["cpu_percent"].mean())
                mems.append(df["mem_percent"].mean())
                rtfs.append(df["rtf"].mean())
                latencies.append(df["avg_ros_latency_ms"].mean())
            
            # Read manifest for failures
            manifest_path = os.path.join(run_dir, "manifest.json")
            m = read_json(manifest_path)
            if m is not None:
                failures.append(m["execution_summary"]["process_failure_count"])

        if cpus:
            scalability_data.append({
                "N": N,
                "cpu_mean": np.mean(cpus), "cpu_ci": compute_ci(cpus),
                "mem_mean": np.mean(mems), "mem_ci": compute_ci(mems),
                "rtf_mean": np.mean(rtfs), "rtf_ci": compute_ci(rtfs),
                "latency_mean": np.mean(latencies), "latency_ci": compute_ci(latencies),
                "failures_mean": np.mean(failures)
            })

    # Save Scalability Table
    scal_df = pd.DataFrame(scalability_data)
    scal_df.to_csv(os.path.join(tables_dir, "scalability.csv"), index=False)
    print("  Saved scalability table to results/tables/scalability.csv")

    # Generate Scalability Plot
    if not scal_df.empty:
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle("Soar & Seek Swarm Scalability & Resource Overhead", fontsize=14, fontweight='bold')
        
        # CPU
        axes[0, 0].errorbar(scal_df["N"], scal_df["cpu_mean"], yerr=scal_df["cpu_ci"], fmt='-o', color='#d62728', capsize=5)
        axes[0, 0].set_title("System CPU Utilization")
        axes[0, 0].set_xlabel("Swarm Size N")
        axes[0, 0].set_ylabel("CPU %")
        axes[0, 0].grid(True, linestyle='--')

        # Memory
        axes[0, 1].errorbar(scal_df["N"], scal_df["mem_mean"], yerr=scal_df["mem_ci"], fmt='-s', color='#1f77b4', capsize=5)
        axes[0, 1].set_title("System Memory Utilization")
        axes[0, 1].set_xlabel("Swarm Size N")
        axes[0, 1].set_ylabel("Memory %")
        axes[0, 1].grid(True, linestyle='--')

        # RTF
        axes[1, 0].errorbar(scal_df["N"], scal_df["rtf_mean"], yerr=scal_df["rtf_ci"], fmt='-^', color='#2ca02c', capsize=5)
        axes[1, 0].axhline(y=1.0, color='r', linestyle=':', label='Real-Time (RTF=1.0)')
        axes[1, 0].set_title("Real-Time Factor (RTF)")
        axes[1, 0].set_xlabel("Swarm Size N")
        axes[1, 0].set_ylabel("RTF Ratio")
        axes[1, 0].grid(True, linestyle='--')
        axes[1, 0].legend()

        # Latency
        axes[1, 1].errorbar(scal_df["N"], scal_df["latency_mean"], yerr=scal_df["latency_ci"], fmt='-d', color='#9467bd', capsize=5)
        axes[1, 1].set_title("Average ROS 2 Msg Latency")
        axes[1, 1].set_xlabel("Swarm Size N")
        axes[1, 1].set_ylabel("Latency (ms)")
        axes[1, 1].grid(True, linestyle='--')

        plt.tight_layout()
        plt.savefig(os.path.join(figures_dir, "scalability.png"), dpi=300)
        plt.close()
        print("  Generated scalability plot at results/figures/scalability.png")

    # --------------------------------------------------
    # 2. STOCHASTIC RUNS & MISSION METRICS
    # --------------------------------------------------
    print("\nProcessing Stochastic Campaign...")
    stochastic_runs = []
    stochastic_raw = []
    for seed in range(42, 62):
        run_dir = os.path.join(results_dir, "stochastic", f"N_6_seed_{seed}")
        summary = read_json(os.path.join(run_dir, "metrics_summary.json"))
        if summary is not None:
            # Gather average metrics across the 6 UAVs in this run
            socs, therm_times, event_rates, invest_rates = [], [], [], []
            for uav, metrics in summary.items():
                socs.append(metrics["final_soc_pct"])
                therm_times.append(metrics["total_thermalling_time_s"])
                event_rates.append(metrics["detected_event_rate_hz"])
                invest_rates.append(metrics["high_priority_investigation_rate_hz"])
            
            stochastic_runs.append({
                "seed": seed,
                "avg_final_soc": np.mean(socs),
                "avg_thermalling_time_s": np.mean(therm_times),
                "avg_event_rate_hz": np.mean(event_rates),
                "avg_investigation_rate_hz": np.mean(invest_rates)
            })
            stochastic_raw.append({
                "final_socs": socs,
                "thermalling_times": therm_times,
                "event_rates": event_rates,
                "invest_rates": invest_rates
            })

    stoch_df = pd.DataFrame(stochastic_runs)
    stoch_df.to_csv(os.path.join(tables_dir, "stochastic_summary.csv"), index=False)
    print("  Saved stochastic runs summary to results/tables/stochastic_summary.csv")

    # Generate Boxplots
    if stochastic_runs:
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        fig.suptitle("Mission Metrics Distribution Over Stochastic Trials (N=6, R=5)", fontsize=14, fontweight='bold')
        
        # Final SoC
        soc_data = [run["final_socs"] for run in stochastic_raw]
        axes[0].boxplot(soc_data, labels=[str(s) for s in stoch_df["seed"]])
        axes[0].set_title("Final State-of-Charge (SoC)")
        axes[0].set_xlabel("Trial Seed")
        axes[0].set_ylabel("SoC %")
        axes[0].grid(True, linestyle=':')

        # Thermalling Time
        therm_data = [run["thermalling_times"] for run in stochastic_raw]
        axes[1].boxplot(therm_data, labels=[str(s) for s in stoch_df["seed"]])
        axes[1].set_title("Thermal Exploitation Time")
        axes[1].set_xlabel("Trial Seed")
        axes[1].set_ylabel("Time (seconds)")
        axes[1].grid(True, linestyle=':')

        # Investigation Latency/Rate
        rate_data = [run["invest_rates"] for run in stochastic_raw]
        axes[2].boxplot(rate_data, labels=[str(s) for s in stoch_df["seed"]])
        axes[2].set_title("HP Investigation Rate")
        axes[2].set_xlabel("Trial Seed")
        axes[2].set_ylabel("Rate (Hz)")
        axes[2].grid(True, linestyle=':')

        plt.tight_layout()
        plt.savefig(os.path.join(figures_dir, "stochastic_distribution.png"), dpi=300)
        plt.close()
        print("  Generated stochastic distributions plot at results/figures/stochastic_distribution.png")

    # --------------------------------------------------
    # 3. THERMAL & EVENT SENSITIVITY CAMPAIGNS
    # --------------------------------------------------
    print("\nProcessing Thermal & Event Sensitivity Campaigns...")
    
    # Thermal Sensitivity
    thermal_sensitivity = []
    # Density conditions: 4 (low), 8 (stochastic base), 16 (high)
    density_map = {
        4: "thermal_sensitivity_low_density",
        8: "stochastic",
        16: "thermal_sensitivity_high_density"
    }

    for density, campaign_folder in density_map.items():
        times = []
        for seed in [42, 43, 44]:
            run_dir = os.path.join(results_dir, campaign_folder, f"N_6_seed_{seed}")
            summary = read_json(os.path.join(run_dir, "metrics_summary.json"))
            if summary is not None:
                for uav, m in summary.items():
                    times.append(m["total_thermalling_time_s"])
        if times:
            thermal_sensitivity.append({
                "density": density,
                "mean_time": np.mean(times),
                "ci_time": compute_ci(times)
            })

    therm_sens_df = pd.DataFrame(thermal_sensitivity)
    therm_sens_df.to_csv(os.path.join(tables_dir, "thermal_sensitivity.csv"), index=False)

    if not therm_sens_df.empty:
        plt.figure(figsize=(6, 4))
        plt.errorbar(therm_sens_df["density"], therm_sens_df["mean_time"], yerr=therm_sens_df["ci_time"], fmt='-o', color='orange', capsize=5, lw=2)
        plt.title("Thermal Density vs Thermal Exploitation Time", fontweight='bold')
        plt.xlabel("Thermal Density (Number of Active Thermals)")
        plt.ylabel("Avg Exploitation Time per UAV (s)")
        plt.grid(True, linestyle='--')
        plt.tight_layout()
        plt.savefig(os.path.join(figures_dir, "thermal_sensitivity.png"), dpi=300)
        plt.close()
        print("  Generated thermal sensitivity plot at results/figures/thermal_sensitivity.png")

    # Event Sensitivity
    # Rates: 0.02 (low), 0.05 (stochastic base), 0.12 (high)
    event_sensitivity = []
    rate_map = {
        0.02: "event_sensitivity_low_load",
        0.05: "stochastic",
        0.12: "event_sensitivity_high_load"
    }

    for rate, campaign_folder in rate_map.items():
        unresolved = []
        latencies = []
        for seed in [42, 43, 44]:
            run_dir = os.path.join(results_dir, campaign_folder, f"N_6_seed_{seed}")
            summary = read_json(os.path.join(run_dir, "metrics_summary.json"))
            if summary is not None:
                for uav, m in summary.items():
                    unresolved.append(m["high_priority_unresolved_count"])
                    if m["mean_investigation_latency_s"] > 0.0:
                        latencies.append(m["mean_investigation_latency_s"])
        if unresolved:
            event_sensitivity.append({
                "event_rate": rate,
                "unresolved_mean": np.mean(unresolved), "unresolved_ci": compute_ci(unresolved),
                "latency_mean": np.mean(latencies) if latencies else 0.0,
                "latency_ci": compute_ci(latencies) if latencies else 0.0
            })

    evt_sens_df = pd.DataFrame(event_sensitivity)
    evt_sens_df.to_csv(os.path.join(tables_dir, "event_sensitivity.csv"), index=False)

    if not evt_sens_df.empty:
        fig, ax1 = plt.subplots(figsize=(7, 4))
        color = '#1f77b4'
        ax1.set_xlabel("Event Spawn Rate (Hz)")
        ax1.set_ylabel("Avg Unresolved Events", color=color)
        ax1.errorbar(evt_sens_df["event_rate"], evt_sens_df["unresolved_mean"], yerr=evt_sens_df["unresolved_ci"], fmt='-o', color=color, capsize=5, lw=2)
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.grid(True, linestyle='--')

        ax2 = ax1.twinx()
        color = '#d62728'
        ax2.set_ylabel("Avg Investigation Latency (s)", color=color)
        ax2.errorbar(evt_sens_df["event_rate"], evt_sens_df["latency_mean"], yerr=evt_sens_df["latency_ci"], fmt='-s', color=color, capsize=5, lw=2)
        ax2.tick_params(axis='y', labelcolor=color)

        plt.title("Event Spawn Rate vs Investigation Performance", fontweight='bold')
        fig.tight_layout()
        plt.savefig(os.path.join(figures_dir, "event_sensitivity.png"), dpi=300)
        plt.close()
        print("  Generated event sensitivity plot at results/figures/event_sensitivity.png")

    # --------------------------------------------------
    # 4. BASELINE & ABLATION PERFORMANCE COMPARISON
    # --------------------------------------------------
    print("\nProcessing Baselines and Ablations...")
    comparison_data = []
    configs = {
        "Baseline (Coverage-Path Only)": "baseline_coverage",
        "Ablation (Soaring Only)": "ablation_soaring_only",
        "Ablation (Events Only)": "ablation_event_only",
        "Full Soar & Seek Framework": "stochastic"
    }

    for label, campaign_folder in configs.items():
        socs = []
        therm_times = []
        energy_patrol = []
        energy_invest = []
        energy_soar = []
        raw_lds = []
        
        for seed in [42, 43, 44]:
            run_dir = os.path.join(results_dir, campaign_folder, f"N_6_seed_{seed}")
            summary = read_json(os.path.join(run_dir, "metrics_summary.json"))
            if summary is not None:
                for uav, m in summary.items():
                    socs.append(m["final_soc_pct"])
                    therm_times.append(m["total_thermalling_time_s"])
                    energy_patrol.append(m["propulsion_energy_wh"].get("PATROL", 0.0))
                    energy_invest.append(m["propulsion_energy_wh"].get("EVENT_INVESTIGATION", 0.0))
                    energy_soar.append(m["propulsion_energy_wh"].get("THERMAL_EXPLOITATION", 0.0))
                    raw_lds.append(m["raw_ld_jsbsim"])

        if socs:
            comparison_data.append({
                "Configuration": label,
                "final_soc_mean": np.mean(socs), "final_soc_ci": compute_ci(socs),
                "thermalling_time_mean": np.mean(therm_times), "thermalling_time_ci": compute_ci(therm_times),
                "energy_patrol_mean": np.mean(energy_patrol),
                "energy_invest_mean": np.mean(energy_invest),
                "energy_soar_mean": np.mean(energy_soar),
                "avg_jsbsim_ld": np.mean(raw_lds),
                "avg_eagle_ld": 12.0
            })

    comp_df = pd.DataFrame(comparison_data)
    comp_df.to_csv(os.path.join(tables_dir, "baseline_comparison.csv"), index=False)
    print("  Saved baseline comparison table to results/tables/baseline_comparison.csv")

    # Generate Performance Bar Chart
    if not comp_df.empty:
        plt.figure(figsize=(10, 5))
        x = np.arange(len(comp_df))
        width = 0.35

        # Plot energy by modes stacked
        p1 = plt.bar(x, comp_df["energy_patrol_mean"], width, label='Patrol Energy (Wh)', color='#1f77b4')
        p2 = plt.bar(x, comp_df["energy_invest_mean"], width, bottom=comp_df["energy_patrol_mean"], label='Investigate Energy (Wh)', color='#ff7f0e')
        p3 = plt.bar(x, comp_df["energy_soar_mean"], width, bottom=comp_df["energy_patrol_mean"] + comp_df["energy_invest_mean"], label='Soaring Energy (Wh)', color='#2ca02c')

        plt.title("Propulsion Energy Consumption Breakdown by FSM State", fontsize=12, fontweight='bold')
        plt.xticks(x, comp_df["Configuration"], rotation=15, ha='right')
        plt.ylabel("Energy Consumed (Wh)")
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(figures_dir, "baseline_comparison.png"), dpi=300)
        plt.close()
        print("  Generated comparison bar chart at results/figures/baseline_comparison.png")

    print("\n==================================================")
    print(" All tables & figures successfully generated!")
    print(" Output directories: results/tables/ and results/figures/")
    print("==================================================")

if __name__ == "__main__":
    main()
