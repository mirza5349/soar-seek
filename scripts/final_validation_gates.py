#!/usr/bin/env python3
"""Final-validation quality gates for the PX4-JSBSim + landing work.

Writes results/quality/final_validation_gates.csv and .txt. Exits non-zero if
any mandatory gate fails (the final PDF generator refuses to run then).
"""
import os
import sys
import pandas as pd

WS = "/home/px4_sitl/sim_paper"
CSV = os.path.join(WS, "results/csv")
FIG = os.path.join(WS, "results/figures")
Q = os.path.join(WS, "results/quality")
GATES = []


def gate(name, ok, detail=""):
    GATES.append({"gate": name, "passed": bool(ok), "detail": detail})
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" — {detail}" if detail else ""))


def main():
    os.makedirs(Q, exist_ok=True)

    # 1. PX4-JSBSim status not FAIL
    status_p = os.path.join(CSV, "px4_jsbsim_status.txt")
    status = open(status_p).readline().strip() if os.path.exists(status_p) else "MISSING"
    gate("px4_jsbsim_status_not_fail", status in
         ("PASS", "PASS_WITH_DOCUMENTED_STARTUP_TRANSIENTS"), f"status={status}")

    # 2. anomaly records available
    anom_p = os.path.join(CSV, "px4_jsbsim_anomalies.csv")
    gate("px4_jsbsim_anomaly_records_available", os.path.exists(anom_p),
         "px4_jsbsim_anomalies.csv present" if os.path.exists(anom_p) else "missing")

    # 3. >= 5 valid landing trials
    runs_p = os.path.join(CSV, "landing_validation_runs.csv")
    n_valid = 0
    if os.path.exists(runs_p):
        lr = pd.read_csv(runs_p)
        n_valid = int(lr["landing_entry"].sum()) if "landing_entry" in lr else 0
    gate("at_least_5_valid_landing_trials", n_valid >= 5, f"{n_valid} valid trials")

    # 4 & 5. touchdown reported separately from landing-state entry; claim matches result
    sum_p = os.path.join(CSV, "landing_validation_summary.csv")
    claim_ok, sep_ok, detail = False, False, "summary missing"
    if os.path.exists(sum_p):
        s = pd.read_csv(sum_p).iloc[0]
        sep_ok = ("landing_entry_rate_pct" in s.index and
                  "touchdown_success_rate_pct" in s.index and
                  "landing_completion_rate_pct" in s.index)
        td = s.get("touchdown_success_rate_pct", float('nan'))
        # claim consistency: a claim_level file must reflect whether touchdown was achieved
        claim_p = os.path.join(CSV, "landing_claim_level.txt")
        claim = open(claim_p).read().strip() if os.path.exists(claim_p) else ""
        if td and td > 0:
            claim_ok = "full_landing_cycle" in claim
        else:
            claim_ok = "trajectory_and_state_execution_only" in claim
        detail = f"touchdown={td}%, claim='{claim}'"
    gate("touchdown_reported_separately_from_entry", sep_ok)
    gate("landing_claim_matches_result", claim_ok, detail)

    # 6. all new figures backed by CSV
    pairs = [("px4_jsbsim_consistency", "px4_jsbsim_consistency.csv"),
             ("landing_trajectory", "landing_validation_runs.csv"),
             ("landing_altitude_distance", "landing_validation_runs.csv"),
             ("landing_state_timeline", "landing_validation_runs.csv")]
    missing = []
    for fig, csv in pairs:
        if not (os.path.exists(os.path.join(FIG, fig + ".png"))
                and os.path.exists(os.path.join(FIG, fig + ".pdf"))
                and os.path.exists(os.path.join(CSV, csv))):
            missing.append(fig)
    gate("new_figures_backed_by_csv", not missing, "; ".join(missing))

    # 7. no contradictory verification statements (old FAILED self-consistency row gone)
    fv_p = os.path.join(CSV, "framework_verification.csv")
    contradictory, has_new = True, False
    if os.path.exists(fv_p):
        fv = pd.read_csv(fv_p)
        contradictory = bool((fv["check_name"] == "px4_jsbsim_telemetry_consistency").any())
        has_new = bool((fv["check_name"] == "px4_jsbsim_ekf_vs_fdm").any())
    gate("no_contradictory_verification_statements", (not contradictory) and has_new,
         "old self-consistency row removed; ekf_vs_fdm present")

    df = pd.DataFrame(GATES)
    df.to_csv(os.path.join(Q, "final_validation_gates.csv"), index=False)
    npass = int(df["passed"].sum())
    with open(os.path.join(Q, "final_validation_gates.txt"), "w") as f:
        f.write("SOAR & SEEK — FINAL VALIDATION GATES\n" + "=" * 50 + "\n")
        for g in GATES:
            f.write(f"[{'PASS' if g['passed'] else 'FAIL'}] {g['gate']}"
                    + (f"  — {g['detail']}" if g['detail'] else "") + "\n")
        f.write("=" * 50 + f"\n{npass}/{len(GATES)} gates passed.\n")
    print(f"\n{npass}/{len(GATES)} final-validation gates passed.")
    return 0 if npass == len(GATES) else 1


if __name__ == "__main__":
    sys.exit(main())
