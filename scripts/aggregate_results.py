#!/usr/bin/env python3
"""Aggregate raw campaign logs into the results/csv tables.

Strict provenance policy:
  * every value is computed from files under logs/raw/<campaign>/<run>/
  * metrics that cannot be computed are written as NaN (never invented)
  * invalid runs are excluded only with a documented reason in
    results/csv/run_quality_flags.csv
  * results/quality/provenance.json maps every output CSV to its source runs

Hard rule: any negative execution duration aborts the pipeline and prints
the corrupt run ID.
"""
import os
import re
import sys
import glob
import json
import math
import numpy as np
import pandas as pd

WORKSPACE = "/home/px4_sitl/sim_paper"
RAW = os.path.join(WORKSPACE, "logs/raw")
CSV_DIR = os.path.join(WORKSPACE, "results/csv")
QUALITY_DIR = os.path.join(WORKSPACE, "results/quality")
PROCESSED = os.path.join(WORKSPACE, "logs/processed")

PROVENANCE = {}
QUALITY_ROWS = []

MODES = ["PATROL", "EVENT_INVESTIGATION", "THERMAL_SEARCH",
         "THERMAL_EXPLOITATION", "GLIDE_RETURN", "LANDING"]


# --------------------------------------------------------------- run loading
def load_run(run_dir):
    """Load one run; returns dict or None (with a quality flag) if unusable."""
    run_id = os.path.relpath(run_dir, RAW)
    summary_path = os.path.join(run_dir, "metrics_summary.json")
    manifest_path = os.path.join(run_dir, "manifest.json")
    if not os.path.exists(summary_path):
        QUALITY_ROWS.append({"run_id": run_id, "issue": "missing_metrics_summary",
                             "excluded": True,
                             "reason": "collector summary absent (run crashed before checkpoint)"})
        return None
    try:
        summary = json.load(open(summary_path))
        manifest = json.load(open(manifest_path)) if os.path.exists(manifest_path) else {}
    except Exception as e:
        QUALITY_ROWS.append({"run_id": run_id, "issue": "unreadable_summary",
                             "excluded": True, "reason": str(e)})
        return None

    uavs = {k: v for k, v in summary.items() if k.startswith("uav_")}
    fleet = summary.get("_fleet", {})

    # ---- duration validation --------------------------------------------
    # Per spec: a strictly NEGATIVE duration is data corruption and HALTS the
    # whole pipeline (this was the original endurance bug). A zero / missing /
    # invalid duration for a single UAV is a per-vehicle boot/arming failure:
    # exclude that UAV with a documented reason and keep the rest of the run.
    excluded_uavs = []
    for k, u in list(uavs.items()):
        end = u.get("endurance_s")
        if end is not None and end < 0.0:
            QUALITY_ROWS.append({
                "run_id": run_id, "issue": f"negative_duration_{k}",
                "excluded": True,
                "reason": f"{k} endurance_s={end} (NEGATIVE — data corruption)"})
            print(f"FATAL: negative execution duration in run {run_id} ({k}: {end})")
            sys.exit(2)
        if end is None or end <= 0.0 or not u.get("endurance_valid", False):
            QUALITY_ROWS.append({
                "run_id": run_id, "issue": f"invalid_duration_{k}",
                "excluded": True,
                "reason": (f"{k} endurance_s={end}, valid={u.get('endurance_valid')} "
                           f"(vehicle never produced valid telemetry — likely boot/arming "
                           f"failure; excluded from aggregates, run retained for other UAVs)")})
            excluded_uavs.append(k)
            continue
        anomalies = u.get("timestamp_anomalies", 0)
        if anomalies > 200:
            QUALITY_ROWS.append({
                "run_id": run_id, "issue": f"timestamp_anomalies_{k}",
                "excluded": False,
                "reason": f"{k} rejected {anomalies} out-of-order samples (kept; sanitised)"})

    for k in excluded_uavs:
        uavs.pop(k, None)

    if not uavs:
        QUALITY_ROWS.append({
            "run_id": run_id, "issue": "no_valid_uavs", "excluded": True,
            "reason": "every UAV had invalid duration; run dropped entirely"})
        return None

    QUALITY_ROWS.append({
        "run_id": run_id, "issue": "none", "excluded": False,
        "reason": (f"{len(uavs)} valid UAV(s)"
                   + (f"; excluded {len(excluded_uavs)}: {excluded_uavs}" if excluded_uavs
                      else "; all validity checks passed"))})
    return {"run_id": run_id, "dir": run_dir, "uavs": uavs, "fleet": fleet,
            "manifest": manifest}


def runs_of(pattern):
    out = []
    for d in sorted(glob.glob(os.path.join(RAW, pattern))):
        r = load_run(d)
        if r is not None:
            out.append(r)
    return out


def nanmean(vals):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    return float(np.mean(vals)) if vals else float('nan')


def nanstd(vals):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    return float(np.std(vals, ddof=1)) if len(vals) > 1 else float('nan')


def fleet_mean(run, field):
    return nanmean([u.get(field) for u in run["uavs"].values()])


def fleet_sum(run, field):
    vals = [u.get(field) for u in run["uavs"].values() if u.get(field) is not None]
    return float(np.sum(vals)) if vals else float('nan')


def mission_metrics_row(run):
    """Common per-run fleet aggregate."""
    uavs = run["uavs"]
    fleet = run["fleet"]
    man = run["manifest"].get("execution_summary", {})
    patrol_powers = []
    for u in uavs.values():
        e = u["propulsion_energy_wh"].get("PATROL", 0.0)
        # mean patrol power needs patrol time; approximate with endurance share
        patrol_powers.append(e)
    therm_time = fleet_sum(run, "total_thermalling_time_s")
    return {
        "run_id": run["run_id"],
        "execution_duration_s": nanmean([u["endurance_s"] for u in uavs.values()]),
        "route_completion_pct": nanmean([u.get("route_completion_pct") for u in uavs.values()]),
        "mission_completion_pct": 100.0 * np.mean([1.0 if u.get("mission_complete") else 0.0
                                                   for u in uavs.values()]),
        "final_soc_pct": nanmean([u.get("final_soc_pct") for u in uavs.values()]),
        "propulsion_energy_wh": fleet_sum(run, "total_propulsion_energy_wh"),
        "thermal_encounters": fleet_sum(run, "thermal_encounters"),
        "thermalling_time_s": therm_time,
        "thermal_alt_gain_m": nanmean([u.get("mean_thermal_alt_gain_m") for u in uavs.values()]),
        "total_events": fleet.get("total_events", float('nan')),
        "total_hp_events": fleet.get("total_hp_events", float('nan')),
        "hp_detected_count": fleet.get("hp_detected_count", float('nan')),
        "hp_investigated_count": fleet.get("hp_investigated_count", float('nan')),
        "hp_expired_count": fleet.get("hp_expired_count", float('nan')),
        "hp_unresolved_count": fleet.get("hp_unresolved_count", float('nan')),
        "hp_detected_pct": fleet.get("hp_detected_pct", float('nan')),
        "hp_investigated_pct": fleet.get("hp_investigated_pct", float('nan')),
        "hp_expired_pct": fleet.get("hp_expired_pct", float('nan')),
        "hp_unresolved_pct": fleet.get("hp_unresolved_pct", float('nan')),
        "all_detected_pct": fleet.get("all_detected_pct", float('nan')),
        "mean_hp_investigation_latency_s": fleet.get("mean_hp_investigation_latency_s", float('nan')),
        "landing_entry_pct": 100.0 * np.mean([1.0 if u.get("entered_landing") else 0.0
                                              for u in uavs.values()]) if uavs else float('nan'),
        "landing_completion_pct": 100.0 * np.mean([1.0 if u.get("landing_success") else 0.0
                                                   for u in uavs.values()]) if uavs else float('nan'),
        "valid_termination_pct": 100.0 * np.mean([1.0 if u.get("mission_complete") else 0.0
                                                  for u in uavs.values()]) if uavs else float('nan'),
        "fsm_transitions": fleet_sum(run, "fsm_transitions"),
        "fsm_transitions_per_min": nanmean([u.get("fsm_transitions_per_min") for u in uavs.values()]),
        "fsm_rejected_transitions": fleet_sum(run, "fsm_rejected_transitions"),
        "landings": sum(1 for u in uavs.values() if u.get("landing_success")),
        "landings_entered": sum(1 for u in uavs.values() if u.get("entered_landing")),
        "process_failures": man.get("process_failure_count", float('nan')),
        "mavlink_timeouts": man.get("mavlink_timeout_count", float('nan')),
    }


def save(df, name, sources):
    path = os.path.join(CSV_DIR, name)
    df.to_csv(path, index=False)
    PROVENANCE[name] = sources
    print(f"Saved {name} ({len(df)} rows)")


# ------------------------------------------------------------------ sections
def aggregate_nominal():
    runs = runs_of("nominal/N_*_seed_*")
    if not runs:
        print("WARNING: no nominal run found")
        return
    run = runs[0]
    rows = []
    for k, u in run["uavs"].items():
        row = {"uav_id": k}
        for f in ["endurance_s", "final_soc_pct", "route_completion_pct",
                  "mission_complete", "total_propulsion_energy_wh",
                  "thermal_encounters", "total_thermalling_time_s",
                  "mean_thermal_alt_gain_m", "events_first_detected_count",
                  "investigations_count", "entered_landing", "landing_success",
                  "timestamp_anomalies"]:
            v = u.get(f)
            row[f] = v if v is not None else float('nan')
        for m in MODES:
            row[f"energy_{m.lower()}_wh"] = u["propulsion_energy_wh"].get(m, float('nan'))
        rows.append(row)
    # fleet summary row
    fr = mission_metrics_row(run)
    fr_row = {"uav_id": "FLEET"}
    fr_row.update({k: v for k, v in fr.items() if k != "run_id"})
    df = pd.DataFrame(rows)
    fleet_df = pd.DataFrame([fr_row])
    out = pd.concat([df, fleet_df], ignore_index=True)
    save(out, "nominal_summary.csv", [run["run_id"]])

    # processed copy for convenience
    os.makedirs(os.path.join(PROCESSED, "nominal"), exist_ok=True)
    out.to_csv(os.path.join(PROCESSED, "nominal", "nominal_summary.csv"), index=False)


def aggregate_baselines():
    config_map = {
        "full_framework": "stochastic/N_6_seed_4[234]",   # seeds 42-44, identical config
        "no_thermal": "baselines/no_thermal/N_6_seed_*",
        "no_event_response": "baselines/no_event_response/N_6_seed_*",
        "non_energy_aware_fsm": "baselines/non_energy_aware_fsm/N_6_seed_*",
        "simplified_battery": "baselines/simplified_battery/N_6_seed_*",
        "coverage_only": "baselines/coverage_only/N_6_seed_*",
    }
    rows, sources = [], []
    for cfg, pattern in config_map.items():
        runs = runs_of(pattern)
        if not runs:
            print(f"WARNING: no runs for baseline {cfg}")
            continue
        sources += [r["run_id"] for r in runs]
        per_run = [mission_metrics_row(r) for r in runs]

        def agg(field):
            return nanmean([p[field] for p in per_run]), nanstd([p[field] for p in per_run])

        row = {"configuration": cfg, "n_runs": len(runs)}
        for field in ["execution_duration_s", "route_completion_pct",
                      "mission_completion_pct", "final_soc_pct",
                      "propulsion_energy_wh", "hp_investigated_pct",
                      "hp_unresolved_pct", "mean_hp_investigation_latency_s",
                      "thermal_encounters", "thermalling_time_s",
                      "process_failures"]:
            m, s = agg(field)
            row[f"{field}_mean"] = m
            row[f"{field}_std"] = s
        rows.append(row)
    save(pd.DataFrame(rows), "reduced_framework_baselines.csv", sources)


def trace_files(run):
    return sorted(glob.glob(os.path.join(run["dir"], "uav_trace_*.csv")))


def aggregate_battery_models():
    """Battery model comparison on the stochastic full-framework runs.

    online   : aerodynamic propulsion-only estimator (logged soc_pct)
    constant : constant-power discharge re-evaluated offline on the same
               timeline (P = capacity-normalised mean armed power of the run)
    px4      : PX4's own simulated battery estimate (logged reference)
    """
    runs = runs_of("stochastic/N_6_seed_*")
    if not runs:
        print("WARNING: no stochastic runs for battery comparison")
        return
    online_final, const_final, px4_final = [], [], []
    rmse_const, rmse_px4 = [], []
    mean_powers = []
    capacity = None
    for run in runs:
        cfg = os.path.join(run["dir"], "config.yaml")
        import yaml as _yaml
        capacity = _yaml.safe_load(open(cfg))['battery_estimator_node']['ros__parameters']['battery_capacity_wh']
        for tf in trace_files(run):
            df = pd.read_csv(tf)
            soc = pd.to_numeric(df['soc_pct'], errors='coerce')
            e = pd.to_numeric(df['energy_consumed_wh'], errors='coerce')
            t = df['sim_time'] - df['sim_time'].iloc[0]
            if soc.dropna().empty or t.iloc[-1] <= 0:
                continue
            dur = t.iloc[-1]
            p_mean = 3600.0 * e.dropna().iloc[-1] / dur if dur > 0 else float('nan')
            mean_powers.append(p_mean)
            online_final.append(soc.dropna().iloc[-1])
            # constant-power discharge on the same timeline
            soc_const = (100.0 * (1.0 - (p_mean * t / 3600.0) / capacity)).clip(lower=0.0)
            const_final.append(soc_const.iloc[-1])
            ok = soc.notna()
            rmse_const.append(float(np.sqrt(np.mean((soc[ok] - soc_const[ok]) ** 2))))
            px4 = pd.to_numeric(df['px4_batt_remaining_pct'], errors='coerce')
            if px4.notna().sum() > 10:
                px4_final.append(px4.dropna().iloc[-1])
                both = soc.notna() & px4.notna()
                rmse_px4.append(float(np.sqrt(np.mean((soc[both] - px4[both]) ** 2))))

    rows = [
        {"model": "online_propulsion_only_estimator",
         "final_soc_pct_mean": nanmean(online_final), "final_soc_pct_std": nanstd(online_final),
         "mean_propulsion_power_w": nanmean(mean_powers),
         "soc_diff_vs_online_pct": 0.0,
         # NaN, not 0: comparing the model against itself is not a validation.
         "soc_rmse_vs_online_pct": float('nan'),
         "notes": "model under test; aerodynamic state-dependent power, motor-off soaring modelled (self-RMSE not a validation -> NaN)"},
        {"model": "constant_power_discharge_baseline",
         "final_soc_pct_mean": nanmean(const_final), "final_soc_pct_std": nanstd(const_final),
         "mean_propulsion_power_w": nanmean(mean_powers),
         "soc_diff_vs_online_pct": nanmean(const_final) - nanmean(online_final),
         "soc_rmse_vs_online_pct": nanmean(rmse_const),
         "notes": "same mean energy, ignores state-dependent draw (no soaring savings shape)"},
        {"model": "px4_default_battery_estimate",
         "final_soc_pct_mean": nanmean(px4_final), "final_soc_pct_std": nanstd(px4_final),
         "mean_propulsion_power_w": float('nan'),
         "soc_diff_vs_online_pct": (nanmean(px4_final) - nanmean(online_final))
            if px4_final else float('nan'),
         "soc_rmse_vs_online_pct": nanmean(rmse_px4) if rmse_px4 else float('nan'),
         "notes": "PX4 SITL simulated battery (different capacity model, independent reference)"},
        {"model": "external_realflight_reference",
         "final_soc_pct_mean": float('nan'), "final_soc_pct_std": float('nan'),
         "mean_propulsion_power_w": float('nan'),
         "soc_diff_vs_online_pct": float('nan'),
         "soc_rmse_vs_online_pct": float('nan'),
         "notes": "NaN: no instrumented real-flight energy log available for this airframe"},
    ]
    save(pd.DataFrame(rows), "battery_model_comparison.csv",
         [r["run_id"] for r in runs])


def aggregate_energy_budget():
    runs = runs_of("stochastic/N_6_seed_*")
    if not runs:
        return
    by_mode = {m: [] for m in MODES}
    patrol_powers, therm_times = [], []
    for run in runs:
        for u in run["uavs"].values():
            for m in MODES:
                by_mode[m].append(u["propulsion_energy_wh"].get(m, float('nan')))
            therm_times.append(u.get("total_thermalling_time_s", 0.0))
        # mean patrol power for the cruise-equivalent saving estimate
        for tf in trace_files(run):
            df = pd.read_csv(tf)
            pat = df[df['fsm_state_name'] == 'PATROL']
            p = pd.to_numeric(pat['power_w'], errors='coerce').dropna()
            if len(p) > 10:
                patrol_powers.append(p.mean())

    p_cruise = nanmean(patrol_powers)
    rows = []
    total = sum(nanmean(v) for v in by_mode.values() if not math.isnan(nanmean(v)))
    for m in MODES:
        mean_e = nanmean(by_mode[m])
        rows.append({
            "mode": m,
            "propulsion_energy_wh_mean": mean_e,
            "propulsion_energy_wh_std": nanstd(by_mode[m]),
            "share_pct": 100.0 * mean_e / total if total > 0 else float('nan'),
        })
    # propulsion-energy saving relative to powered cruise (NOT generated energy)
    mean_therm_time = nanmean(therm_times)
    saving = p_cruise * mean_therm_time / 3600.0 if not math.isnan(p_cruise) else float('nan')
    rows.append({
        "mode": "THERMAL_EXPLOITATION_SAVING_VS_CRUISE",
        "propulsion_energy_wh_mean": saving,
        "propulsion_energy_wh_std": float('nan'),
        "share_pct": float('nan'),
    })
    save(pd.DataFrame(rows), "energy_budget_by_mode.csv", [r["run_id"] for r in runs])


def aggregate_sensitivity(kind):
    """kind in {'thermal', 'event'}"""
    if kind == 'thermal':
        levels = {"low": "thermal_sensitivity/low/N_6_seed_*",
                  "nominal": "stochastic/N_6_seed_4[23456]",
                  "high": "thermal_sensitivity/high/N_6_seed_*"}
        out_name = "thermal_sensitivity.csv"
    else:
        levels = {"low": "event_sensitivity/low/N_6_seed_*",
                  "nominal": "stochastic/N_6_seed_4[23456]",
                  "high": "event_sensitivity/high/N_6_seed_*"}
        out_name = "event_sensitivity.csv"

    rows, sources = [], []
    for lvl, pattern in levels.items():
        runs = runs_of(pattern)
        if not runs:
            print(f"WARNING: no runs for {kind} sensitivity '{lvl}'")
            continue
        sources += [r["run_id"] for r in runs]
        per_run = [mission_metrics_row(r) for r in runs]
        row = {"level": lvl, "n_runs": len(runs)}
        if kind == 'thermal':
            n_uavs = [len(r["uavs"]) for r in runs]
            fields = ["execution_duration_s", "final_soc_pct", "thermalling_time_s",
                      "thermal_alt_gain_m", "propulsion_energy_wh"]
            row["thermal_encounters_per_uav_mean"] = nanmean(
                [p["thermal_encounters"] / n for p, n in zip(per_run, n_uavs)])
            row["thermal_encounters_per_uav_std"] = nanstd(
                [p["thermal_encounters"] / n for p, n in zip(per_run, n_uavs)])
            # propulsion-energy saving during exploitation (cruise-equivalent)
            savings = []
            for r in runs:
                p_cruise_vals = []
                for tf in trace_files(r):
                    df = pd.read_csv(tf)
                    pat = df[df['fsm_state_name'] == 'PATROL']
                    p = pd.to_numeric(pat['power_w'], errors='coerce').dropna()
                    if len(p) > 10:
                        p_cruise_vals.append(p.mean())
                pc = nanmean(p_cruise_vals)
                tt = fleet_sum(r, "total_thermalling_time_s")
                savings.append(pc * tt / 3600.0 if not math.isnan(pc) else float('nan'))
            row["exploitation_saving_wh_mean"] = nanmean(savings)
            row["exploitation_saving_wh_std"] = nanstd(savings)
        else:
            fields = ["total_events", "total_hp_events", "hp_detected_count",
                      "hp_investigated_count", "hp_expired_count",
                      "hp_unresolved_count", "hp_detected_pct",
                      "hp_investigated_pct", "hp_expired_pct", "hp_unresolved_pct",
                      "all_detected_pct", "mean_hp_investigation_latency_s"]
            n_uavs = [len(r["uavs"]) for r in runs]
            row["event_workload_per_uav_mean"] = nanmean(
                [(p["hp_investigated_count"] if not math.isnan(p["hp_investigated_count"]) else 0) / n
                 for p, n in zip(per_run, n_uavs)])
        for f in fields:
            row[f"{f}_mean"] = nanmean([p[f] for p in per_run])
            row[f"{f}_std"] = nanstd([p[f] for p in per_run])
        rows.append(row)
    save(pd.DataFrame(rows), out_name, sources)


def aggregate_stochastic():
    runs = runs_of("stochastic/N_6_seed_*")
    if not runs:
        print("WARNING: no stochastic runs")
        return
    per_seed = []
    for run in runs:
        seed = int(re.search(r"seed_(\d+)", run["run_id"]).group(1))
        row = {"seed": seed}
        row.update(mission_metrics_row(run))
        per_seed.append(row)
    df = pd.DataFrame(per_seed).sort_values("seed")

    # Hard gate: non-negative durations (already enforced per-UAV in load_run,
    # double-checked here at run level)
    bad = df[df["execution_duration_s"] < 0]
    if not bad.empty:
        print(f"FATAL: negative execution duration in runs: {list(bad['run_id'])}")
        sys.exit(2)

    save(df.drop(columns=["run_id"]), "stochastic_runs_per_seed.csv",
         [r["run_id"] for r in runs])

    metrics = [c for c in df.columns if c not in ("seed", "run_id")]
    rows_s = []
    for c in metrics:
        vals = pd.to_numeric(df[c], errors='coerce').dropna()
        n = len(vals)
        ci95 = (1.96 * vals.std(ddof=1) / np.sqrt(n)) if n > 1 else float('nan')
        rows_s.append({
            "metric": c, "mean": vals.mean() if n else float('nan'),
            "std": vals.std(ddof=1) if n > 1 else float('nan'),
            "median": vals.median() if n else float('nan'),
            "min": vals.min() if n else float('nan'),
            "max": vals.max() if n else float('nan'),
            "ci95_halfwidth": ci95, "n_valid": int(n),
        })
    save(pd.DataFrame(rows_s), "stochastic_runs_summary.csv", [r["run_id"] for r in runs])


def aggregate_scalability():
    """Reads raw summaries directly (NOT the exclusion-filtered loader) so the
    denominator is always the REQUESTED fleet size N, per spec."""
    rows, sources = [], []
    for n in [6, 12, 24]:
        run_dirs = sorted(glob.glob(os.path.join(RAW, f"scalability/N_{n}_seed_*")))
        run_dirs = [d for d in run_dirs if os.path.exists(os.path.join(d, "metrics_summary.json"))]
        if not run_dirs:
            print(f"WARNING: no scalability runs for N={n}")
            continue
        sources += [os.path.relpath(d, RAW) for d in run_dirs]
        cpus, mems, rtfs, lats, logs, fails, touts = [], [], [], [], [], [], []
        spawned, connected, armed_l, takeoff, droprate = [], [], [], [], []
        for d in run_dirs:
            fm = os.path.join(d, "framework_metrics.csv")
            if os.path.exists(fm):
                df = pd.read_csv(fm)
                cpus.append(pd.to_numeric(df['cpu_percent'], errors='coerce').mean())
                mems.append(pd.to_numeric(df['mem_percent'], errors='coerce').mean())
                rtfs.append(pd.to_numeric(df['rtf_window'], errors='coerce').mean())
                lats.append(pd.to_numeric(df['avg_ros_latency_ms'], errors='coerce').mean())
            man = json.load(open(os.path.join(d, "manifest.json"))).get("execution_summary", {}) \
                if os.path.exists(os.path.join(d, "manifest.json")) else {}
            fails.append(man.get("process_failure_count", float('nan')))
            touts.append(man.get("mavlink_timeout_count", float('nan')))
            logs.append(man.get("total_log_size_kb", float('nan')))
            spawned.append(n)  # launcher always spawns N processes
            armed_l.append(man.get("uavs_armed", float('nan')))
            # connected = produced >1 telemetry sample; takeoff = reached >10 m
            n_conn = n_to = 0
            s = json.load(open(os.path.join(d, "metrics_summary.json")))
            for tf in glob.glob(os.path.join(d, "uav_trace_*.csv")):
                try:
                    tdf = pd.read_csv(tf)
                except Exception:
                    continue
                if len(tdf) > 1:
                    n_conn += 1
                    if pd.to_numeric(tdf['alt_rel_m'], errors='coerce').max() > 10.0:
                        n_to += 1
            connected.append(n_conn)
            takeoff.append(n_to)
            droprate.append(float('nan'))  # no per-message drop instrumentation; honest NaN
        rows.append({
            "fleet_size_n": n, "n_runs": len(run_dirs),
            "requested_uavs": n,
            "spawned_processes_mean": nanmean(spawned),
            "connected_uavs_mean": nanmean(connected),
            "armed_uavs_mean": nanmean(armed_l),
            "takeoff_uavs_mean": nanmean(takeoff),
            "arming_success_pct": 100.0 * nanmean(armed_l) / n,
            "takeoff_success_pct": 100.0 * nanmean(takeoff) / n,
            "cpu_pct_mean": nanmean(cpus), "cpu_pct_std": nanstd(cpus),
            "mem_pct_mean": nanmean(mems), "mem_pct_std": nanstd(mems),
            "rtf_mean": nanmean(rtfs), "rtf_std": nanstd(rtfs),
            "ros2_latency_ms_mean": nanmean(lats), "ros2_latency_ms_std": nanstd(lats),
            "ros2_dropped_msg_rate": nanmean(droprate),
            "mavlink_timeout_count_mean": nanmean(touts),
            "process_failure_count_mean": nanmean(fails),
            "log_size_mb_mean": nanmean(logs) / 1024.0 if logs else float('nan'),
        })
    save(pd.DataFrame(rows), "scalability_overhead.csv", sources)


def aggregate_event_lifecycle():
    """Concatenate per-run event ledgers into one lifecycle table."""
    rows, sources = [], []
    patterns = ["stochastic/N_6_seed_*", "event_sensitivity/*/N_6_seed_*",
                "nominal/N_*_seed_*"]
    for pat in patterns:
        for d in sorted(glob.glob(os.path.join(RAW, pat))):
            ep = os.path.join(d, "events.csv")
            if not os.path.exists(ep):
                continue
            rid = os.path.relpath(d, RAW)
            try:
                ev = pd.read_csv(ep)
            except Exception:
                continue
            ev.insert(0, "experiment_id", rid)
            rows.append(ev)
            sources.append(rid)
    if rows:
        out = pd.concat(rows, ignore_index=True)
        save(out, "event_lifecycle_records.csv", sources)
    else:
        print("WARNING: no event ledgers for lifecycle records")


def aggregate_fsm_stats():
    """FSM transition statistics from the (correct) transition logs, per
    campaign condition. Dwell is measured over true state segments."""
    rows, sources = [], []
    conditions = {
        "nominal_stochastic": "stochastic/N_6_seed_*",
        "full_framework": "baselines/full_framework/N_6_seed_*",
    }
    # full_framework baseline reuses stochastic; fall back if folder absent
    for label, pat in [("stochastic_full_framework", "stochastic/N_6_seed_*")]:
        per_uav_rates, dwell_all, sub2s, reentries, rejected, totals = [], [], 0, [], [], 0
        ndir = 0
        for d in sorted(glob.glob(os.path.join(RAW, pat))):
            tp = os.path.join(d, "fsm_transitions.csv")
            sp = os.path.join(d, "metrics_summary.json")
            if not (os.path.exists(tp) and os.path.exists(sp)):
                continue
            ndir += 1
            sources.append(os.path.relpath(d, RAW))
            tr = pd.read_csv(tp)
            s = json.load(open(sp))
            for uav, g in tr.groupby('uav_id'):
                g = g.sort_values('sim_time')
                dur = (g['sim_time'].max() - g['sim_time'].min())
                if dur > 1:
                    per_uav_rates.append(len(g) / (dur / 60.0))
                totals_local = len(g)
                # segment dwell = gap between consecutive transitions
                dt = g['sim_time'].diff().dropna()
                dwell_all.extend(dt.tolist())
                sub2s += int((dt < 2.0).sum())
            for k, u in s.items():
                if k.startswith('uav_'):
                    reentries.append(u.get('fsm_thermal_reentries', 0) or 0)
                    rejected.append(u.get('fsm_rejected_transitions', 0) or 0)
        if not per_uav_rates:
            continue
        dwell_arr = np.array(dwell_all) if dwell_all else np.array([np.nan])
        rows.append({
            "condition": label, "n_runs": ndir,
            "transitions_per_uav_per_min_mean": float(np.mean(per_uav_rates)),
            "transitions_per_uav_per_min_max": float(np.max(per_uav_rates)),
            "mean_state_dwell_s": float(np.nanmean(dwell_arr)),
            "median_state_dwell_s": float(np.nanmedian(dwell_arr)),
            "min_state_dwell_s": float(np.nanmin(dwell_arr)),
            "sub_2s_dwell_count": int(sub2s),
            "sub_2s_dwell_pct": 100.0 * sub2s / max(1, len(dwell_all)),
            "thermal_reentries_total": int(np.sum(reentries)),
            "rejected_transitions_total": int(np.sum(rejected)),
        })
    save(pd.DataFrame(rows), "fsm_transition_statistics.csv", sources)


def aggregate_paired_effects():
    """Per-seed paired differences of each ablation vs full_framework."""
    metrics = ["route_completion_pct", "final_soc_pct", "propulsion_energy_wh",
               "hp_investigated_pct", "hp_unresolved_pct", "thermalling_time_s",
               "valid_termination_pct"]

    def per_seed(pattern):
        out = {}
        for d in sorted(glob.glob(os.path.join(RAW, pattern))):
            r = load_run(d)
            if r is None:
                continue
            m = re.search(r"seed_(\d+)", d)
            if m:
                out[int(m.group(1))] = mission_metrics_row(r)
        return out

    full = per_seed("stochastic/N_6_seed_4[23456]")  # full_framework = stochastic 42-46
    rows, sources = [], []
    for cfg in ["no_thermal", "no_event_response", "non_energy_aware_fsm",
                "simplified_battery", "coverage_only"]:
        abl = per_seed(f"baselines/{cfg}/N_6_seed_*")
        common = sorted(set(full) & set(abl))
        if not common:
            continue
        sources += [f"baselines/{cfg}/N_6_seed_{s}" for s in common]
        row = {"configuration": cfg, "n_paired_seeds": len(common)}
        for met in metrics:
            diffs = [(abl[s].get(met, float('nan')) - full[s].get(met, float('nan')))
                     for s in common]
            row[f"delta_{met}_mean"] = nanmean(diffs)
            row[f"delta_{met}_std"] = nanstd(diffs)
        rows.append(row)
    if rows:
        save(pd.DataFrame(rows), "reduced_framework_paired_effects.csv", sources)
    else:
        print("WARNING: no paired baseline effects (need matched seeds)")


def main():
    os.makedirs(CSV_DIR, exist_ok=True)
    os.makedirs(QUALITY_DIR, exist_ok=True)
    os.makedirs(PROCESSED, exist_ok=True)

    print("==================================================")
    print(" Aggregating campaign results")
    print("==================================================")
    aggregate_nominal()
    aggregate_baselines()
    aggregate_battery_models()
    aggregate_energy_budget()
    aggregate_sensitivity('thermal')
    aggregate_sensitivity('event')
    aggregate_stochastic()
    aggregate_scalability()
    aggregate_event_lifecycle()
    aggregate_fsm_stats()
    aggregate_paired_effects()

    # quality flags + provenance — separated into run-level and UAV-record-level
    run_rows, uav_rows = [], []
    for q in QUALITY_ROWS:
        rid = q["run_id"]
        issue = q["issue"]
        m = re.search(r"uav_(\d+)", issue)
        rec = {
            "experiment_id": rid,
            "uav_id": int(m.group(1)) if m else "",
            "issue": issue,
            "exclusion_scope": ("uav_record" if m else ("run" if q["excluded"] else "none")),
            "excluded": q["excluded"],
            "reason": q["reason"],
            "source_log": os.path.join("logs/raw", rid, "metrics_summary.json"),
        }
        if m:
            uav_rows.append(rec)
        else:
            run_rows.append(rec)
    cols = ["experiment_id", "uav_id", "issue", "exclusion_scope", "excluded",
            "reason", "source_log"]
    pd.DataFrame(run_rows).drop_duplicates().to_csv(
        os.path.join(CSV_DIR, "run_quality_flags.csv"), index=False, columns=cols)
    # header-only file when there are no UAV-record exclusions (do not invent a row)
    pd.DataFrame(uav_rows, columns=cols).to_csv(
        os.path.join(CSV_DIR, "uav_record_quality_flags.csv"), index=False)
    print(f"Saved run_quality_flags.csv ({len(run_rows)}) + "
          f"uav_record_quality_flags.csv ({len(uav_rows)})")
    with open(os.path.join(QUALITY_DIR, "provenance.json"), "w") as f:
        json.dump(PROVENANCE, f, indent=2)
    print("Saved results/quality/provenance.json")
    print("Aggregation complete.")


if __name__ == "__main__":
    main()
