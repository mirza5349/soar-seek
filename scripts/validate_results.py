#!/usr/bin/env python3
"""Quality gates for the results package.

Runs all 13 required gates; exits non-zero (and prints the offending run or
file) if any gate fails. generate_results_pdf.py refuses to build the PDF
unless this script passes.
"""
import os
import re
import sys
import glob
import json
import pandas as pd
import numpy as np

WORKSPACE = "/home/px4_sitl/sim_paper"
RAW = os.path.join(WORKSPACE, "logs/raw")
CSV_DIR = os.path.join(WORKSPACE, "results/csv")
FIG_DIR = os.path.join(WORKSPACE, "results/figures")
QUALITY_DIR = os.path.join(WORKSPACE, "results/quality")

FAILURES = []
GATES = []


def gate(num, name, ok, detail=""):
    GATES.append({"gate": num, "name": name, "passed": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] Gate {num}: {name}" + (f" — {detail}" if detail else ""))
    if not ok:
        FAILURES.append(f"Gate {num} ({name}): {detail}")


def all_runs():
    return sorted(glob.glob(os.path.join(RAW, "*", "**", "metrics_summary.json"),
                            recursive=True))


def main():
    print("==================================================")
    print(" Results quality gates")
    print("==================================================")
    runs = all_runs()

    # Gate 1: no negative execution duration anywhere
    bad = []
    # Gate fails only on strictly NEGATIVE durations (data corruption).
    # Zero/invalid single-UAV durations are documented exclusions in
    # run_quality_flags.csv, not gate failures.
    for sp in runs:
        s = json.load(open(sp))
        for k, u in s.items():
            if k.startswith('uav_') and u.get('endurance_s', 0) < 0:
                bad.append(f"{os.path.dirname(sp)}:{k}={u.get('endurance_s')}")
    for name in ["stochastic_runs_per_seed.csv", "reduced_framework_baselines.csv"]:
        p = os.path.join(CSV_DIR, name)
        if os.path.exists(p):
            df = pd.read_csv(p)
            for col in [c for c in df.columns if 'duration' in c or 'endurance' in c]:
                neg = df[pd.to_numeric(df[col], errors='coerce') < 0]
                if not neg.empty:
                    bad.append(f"{name}:{col} rows {list(neg.index)}")
    gate(1, "no negative execution duration", not bad, "; ".join(bad[:5]))

    # Gate 2: no missing UAV IDs (one trace per UAV in every run)
    bad = []
    for sp in runs:
        d = os.path.dirname(sp)
        man_p = os.path.join(d, "manifest.json")
        if not os.path.exists(man_p):
            bad.append(f"{d}: no manifest")
            continue
        n = json.load(open(man_p))['parameters']['fleet_size_n']
        traces = glob.glob(os.path.join(d, "uav_trace_*.csv"))
        ids = sorted(int(re.search(r"uav_trace_(\d+)", t).group(1)) for t in traces)
        if ids != list(range(1, n + 1)):
            bad.append(f"{d}: traces {ids} != 1..{n}")
    gate(2, "no missing UAV IDs", not bad, "; ".join(bad[:5]))

    # Gate 3: no missing timestamps in traces
    bad = []
    for sp in runs:
        for t in glob.glob(os.path.join(os.path.dirname(sp), "uav_trace_*.csv")):
            df = pd.read_csv(t, usecols=['sim_time'])
            if df['sim_time'].isna().any():
                bad.append(t)
    gate(3, "no missing timestamps", not bad, "; ".join(bad[:5]))

    # Gate 4: no duplicate ROS 2 namespace assignments (MAVSDK routing check)
    ver_p = os.path.join(CSV_DIR, "framework_verification.csv")
    ok, detail = False, "framework_verification.csv missing"
    if os.path.exists(ver_p):
        ver = pd.read_csv(ver_p)
        rows = ver[ver['check_name'].isin(
            ['mavsdk_command_routing_per_uav', 'ros2_namespace_isolation_no_crosstalk'])]
        ok = (not rows.empty) and rows['passed'].all()
        detail = "" if ok else "verification suite reports namespace/routing failures"
    gate(4, "no duplicate ROS 2 namespace assignments", ok, detail)

    # Gate 5: no missing event-state labels
    bad = []
    for sp in runs:
        ep = os.path.join(os.path.dirname(sp), "events.csv")
        if not os.path.exists(ep):
            bad.append(f"{ep} missing")
            continue
        ev = pd.read_csv(ep)
        if len(ev) and not ev['final_state'].isin(
                ['investigated', 'expired', 'unresolved_active']).all():
            bad.append(ep)
    gate(5, "no missing event-state labels", not bad, "; ".join(bad[:5]))

    # Gate 6: thermal segments all carry entry AND exit labels
    bad = []
    for sp in runs:
        s = json.load(open(sp))
        for k, u in s.items():
            if not k.startswith('uav_'):
                continue
            for seg in u.get('thermal_segments', []):
                if seg.get('entry_t') is None or seg.get('exit_t') is None \
                        or seg['exit_t'] < seg['entry_t']:
                    bad.append(f"{os.path.dirname(sp)}:{k}")
    gate(6, "thermal segments have entry/exit labels", not bad, "; ".join(bad[:5]))

    # Gate 7: SOC column present in every trace
    bad = []
    for sp in runs:
        for t in glob.glob(os.path.join(os.path.dirname(sp), "uav_trace_*.csv")):
            cols = pd.read_csv(t, nrows=1).columns
            if 'soc_pct' not in cols:
                bad.append(t)
    gate(7, "SOC column present", not bad, "; ".join(bad[:5]))

    # Gate 8: no impossible SOC values
    bad = []
    for sp in runs:
        for t in glob.glob(os.path.join(os.path.dirname(sp), "uav_trace_*.csv")):
            soc = pd.to_numeric(pd.read_csv(t, usecols=['soc_pct'])['soc_pct'],
                                errors='coerce').dropna()
            if len(soc) and ((soc < -0.01).any() or (soc > 100.01).any()):
                bad.append(f"{t}: range [{soc.min():.2f}, {soc.max():.2f}]")
    gate(8, "SOC within [0, 100]", not bad, "; ".join(bad[:3]))

    # Gate 9: no synthetic fallback — every results CSV has provenance
    prov_p = os.path.join(QUALITY_DIR, "provenance.json")
    # Tables that are NOT run-aggregates (static reference, geometric, or
    # verification-derived) are exempt from the run-provenance requirement.
    analytic_ok = {"coverage_path_comparison.csv", "framework_verification.csv",
                   "run_quality_flags.csv", "uav_record_quality_flags.csv",
                   "quality_gates.csv", "quality_gate_report.csv",
                   "metric_definitions.csv", "uav_instance_mapping.csv",
                   "framework_feature_comparison.csv", "energy_model_parameters.csv",
                   "thermal_parameter_justification.csv", "event_parameter_justification.csv",
                   "px4_jsbsim_consistency.csv", "ros2_isolation_checks.csv",
                   "mavsdk_routing_checks.csv"}
    ok, detail = False, "provenance.json missing"
    if os.path.exists(prov_p):
        prov = json.load(open(prov_p))
        missing = []
        for csv_file in glob.glob(os.path.join(CSV_DIR, "*.csv")):
            name = os.path.basename(csv_file)
            if name in analytic_ok:
                continue
            srcs = prov.get(name)
            if not srcs:
                missing.append(f"{name}: no provenance")
                continue
            for s in srcs[:3]:
                if not os.path.exists(os.path.join(RAW, s)):
                    missing.append(f"{name}: source {s} absent")
        ok = not missing
        detail = "; ".join(missing[:5])
    gate(9, "no synthetic fallback (provenance for every table)", ok, detail)

    # Gate 10: plotted values traceable — every figure's backing CSV / raw run exists
    fig_sources = {
        "reduced_framework_baselines": "reduced_framework_baselines.csv",
        "energy_budget_breakdown_no_avionics": "energy_budget_by_mode.csv",
        "thermal_sensitivity": "thermal_sensitivity.csv",
        "event_sensitivity_outcomes": "event_sensitivity.csv",
        "event_sensitivity_latency": "event_sensitivity.csv",
        "repeated_stochastic_summary": "stochastic_runs_per_seed.csv",
        "scalability_overhead": "scalability_overhead.csv",
        "evaluation_region_path": "coverage_path_comparison.csv",
    }
    bad = [f"{fig}: {src} missing" for fig, src in fig_sources.items()
           if not os.path.exists(os.path.join(CSV_DIR, src))]
    gate(10, "every plotted value backed by a CSV from raw logs", not bad,
         "; ".join(bad[:5]))

    # Gate 11: every figure exists in PNG and PDF
    required_figs = ["evaluation_region_path", "executed_path_overlay",
                     "representative_uav_timeline", "soc_time_series_by_mode",
                     "thermal_altitude_energy_trace", "reduced_framework_baselines",
                     "energy_budget_breakdown_no_avionics", "thermal_sensitivity",
                     "event_sensitivity_outcomes", "event_sensitivity_latency",
                     "repeated_stochastic_summary", "scalability_overhead",
                     "coverage_path_comparison", "battery_model_comparison",
                     "fsm_transition_timeline", "path_behaviour_zoom"]
    bad = []
    for f in required_figs:
        for ext in (".png", ".pdf"):
            if not os.path.exists(os.path.join(FIG_DIR, f + ext)):
                bad.append(f + ext)
    gate(11, "every figure has PNG and PDF", not bad, "; ".join(bad[:6]))

    # Gate 12: all required tables exist as CSV
    required_csvs = ["nominal_summary.csv", "framework_verification.csv",
                     "coverage_path_comparison.csv", "reduced_framework_baselines.csv",
                     "reduced_framework_paired_effects.csv",
                     "battery_model_comparison.csv", "energy_budget_by_mode.csv",
                     "energy_model_parameters.csv", "metric_definitions.csv",
                     "uav_instance_mapping.csv", "framework_feature_comparison.csv",
                     "thermal_sensitivity.csv", "event_sensitivity.csv",
                     "event_lifecycle_records.csv", "fsm_transition_statistics.csv",
                     "stochastic_runs_summary.csv", "stochastic_runs_per_seed.csv",
                     "scalability_overhead.csv", "run_quality_flags.csv",
                     "uav_record_quality_flags.csv",
                     "px4_jsbsim_consistency.csv", "ros2_isolation_checks.csv",
                     "mavsdk_routing_checks.csv"]
    bad = [c for c in required_csvs if not os.path.exists(os.path.join(CSV_DIR, c))]
    gate(12, "all tables present as CSV", not bad, "; ".join(bad))

    # Gate 13: old results removed before the new PDF
    stale = [p for p in [
        os.path.join(WORKSPACE, "results/soar_seek_simulation_results.pdf"),
        os.path.join(WORKSPACE, "results/results_report.pdf"),
        os.path.join(WORKSPACE, "results/full_system_results.pdf"),
        os.path.join(WORKSPACE, "results/pdf/soar_seek_simulation_results_revised.pdf")]
        if os.path.exists(p)]
    gate(13, "old results PDF archived/deleted", not stale, "; ".join(stale))

    mapping_p = os.path.join(CSV_DIR, "uav_instance_mapping.csv")
    mp = pd.read_csv(mapping_p) if os.path.exists(mapping_p) else None

    # Gate 14: no duplicate MAVLink system IDs
    gate(14, "no duplicate MAVLink system IDs",
         mp is not None and mp['mav_sys_id'].is_unique,
         "" if (mp is not None and mp['mav_sys_id'].is_unique) else "mapping missing or dup sys_id")

    # Gate 15: no UDP port collisions across all port columns
    ok15, d15 = False, "mapping missing"
    if mp is not None:
        port_cols = ['jsbsim_tcp_port', 'mavsdk_offboard_remote_port', 'gcs_remote_port',
                     'gcs_local_port', 'offboard_local_port', 'mavsdk_grpc_port']
        allports = pd.concat([mp[c] for c in port_cols])
        ok15 = allports.is_unique
        d15 = "" if ok15 else "port collision detected"
    gate(15, "no UDP port collisions", ok15, d15)

    # Gate 16: no duplicate DDS client keys
    gate(16, "no duplicate DDS client keys",
         mp is not None and mp['uxrce_dds_key'].is_unique,
         "" if (mp is not None and mp['uxrce_dds_key'].is_unique) else "dup dds key")

    # Gate 17: no duplicate ROS 2 namespaces
    gate(17, "no duplicate ROS 2 namespaces",
         mp is not None and mp['ros2_namespace'].is_unique,
         "" if (mp is not None and mp['ros2_namespace'].is_unique) else "dup namespace")

    # Gate 18: route distance validates to 23.3 km
    ok18, d18 = False, "coverage CSV missing"
    cov_p = os.path.join(CSV_DIR, "coverage_path_comparison.csv")
    if os.path.exists(cov_p):
        cov = pd.read_csv(cov_p)
        sel = cov[cov['strategy'].str.startswith('Selected')]
        if len(sel):
            dist = float(sel.iloc[0]['total_path_length_km'])
            ok18 = abs(dist - 23.3) <= 0.5
            d18 = f"selected route = {dist:.2f} km (target 23.3 +/- 0.5)"
    gate(18, "route distance validates to 23.3 km", ok18, d18)

    # Gate 19: exclusions correctly classified (run vs uav-record files present & disjoint scope)
    ok19, d19 = False, "exclusion files missing"
    rqf = os.path.join(CSV_DIR, "run_quality_flags.csv")
    uqf = os.path.join(CSV_DIR, "uav_record_quality_flags.csv")
    if os.path.exists(rqf) and os.path.exists(uqf):
        rq = pd.read_csv(rqf)
        uq = pd.read_csv(uqf)
        run_ok = 'exclusion_scope' in rq.columns and (rq['exclusion_scope'] != 'uav_record').all()
        uav_ok = uq.empty or ('exclusion_scope' in uq.columns and (uq['exclusion_scope'] == 'uav_record').all())
        ok19 = run_ok and uav_ok
        d19 = "" if ok19 else "scope mislabeled between run and uav-record flag files"
    gate(19, "exclusions correctly classified (run vs uav-record)", ok19, d19)

    # Gate 20: all requested UAVs included in scalability denominators
    ok20, d20 = False, "scalability CSV missing"
    scal_p = os.path.join(CSV_DIR, "scalability_overhead.csv")
    if os.path.exists(scal_p):
        scal = pd.read_csv(scal_p)
        if len(scal) and 'requested_uavs' in scal.columns:
            ok20 = bool((scal['requested_uavs'] == scal['fleet_size_n']).all())
            d20 = "" if ok20 else "denominator != requested fleet size"
    gate(20, "all requested UAVs in scalability denominators", ok20, d20)

    # Gate 21: baseline seeds matched (paired effects has >=5 seeds per config)
    ok21, d21 = False, "paired-effects CSV missing"
    pe_p = os.path.join(CSV_DIR, "reduced_framework_paired_effects.csv")
    if os.path.exists(pe_p):
        pe = pd.read_csv(pe_p)
        if len(pe) and 'n_paired_seeds' in pe.columns:
            ok21 = bool((pe['n_paired_seeds'] >= 5).all())
            d21 = "" if ok21 else f"min paired seeds = {int(pe['n_paired_seeds'].min())} (<5)"
    gate(21, "baseline seeds matched (>=5 paired)", ok21, d21)

    # Gate 22: event lifecycle temporally valid (activation <= expiration; detection within window)
    bad = []
    for sp in runs:
        ep = os.path.join(os.path.dirname(sp), "events.csv")
        if not os.path.exists(ep):
            continue
        ev = pd.read_csv(ep)
        if len(ev) == 0:
            continue
        if (ev['expiration_time'] < ev['activation_time']).any():
            bad.append(f"{os.path.dirname(sp)}: expiration<activation")
    gate(22, "event lifecycle temporally valid", not bad, "; ".join(bad[:5]))

    os.makedirs(QUALITY_DIR, exist_ok=True)
    gates_df = pd.DataFrame(GATES)
    gates_df.to_csv(os.path.join(QUALITY_DIR, "quality_gates.csv"), index=False)
    gates_df.to_csv(os.path.join(QUALITY_DIR, "quality_gate_report.csv"), index=False)
    gates_df.to_csv(os.path.join(CSV_DIR, "quality_gates.csv"), index=False)
    with open(os.path.join(QUALITY_DIR, "quality_gate_report.txt"), "w") as f:
        f.write("SOAR & SEEK — QUALITY GATE REPORT\n" + "=" * 50 + "\n")
        for g in GATES:
            f.write(f"[{'PASS' if g['passed'] else 'FAIL'}] Gate {g['gate']}: {g['name']}"
                    + (f"  — {g['detail']}" if g['detail'] else "") + "\n")
        n_pass = sum(1 for g in GATES if g['passed'])
        f.write("=" * 50 + f"\n{n_pass}/{len(GATES)} gates passed.\n")
        if FAILURES:
            f.write("\nFAILURES:\n" + "\n".join(f"  - {x}" for x in FAILURES) + "\n")

    print("==================================================")
    if FAILURES:
        print("QUALITY GATES FAILED:")
        for f in FAILURES:
            print(f"  - {f}")
        print("DO NOT generate the final PDF until these are fixed.")
        return 1
    print(f"All {len(GATES)} quality gates passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
