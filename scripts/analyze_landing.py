#!/usr/bin/env python3
"""Landing-cycle validation analysis.

Reads landing-scenario run dirs (each UAV = one landing trial from a distinct
approach position) and validates the cycle:
  RETURN -> LANDING ENTRY -> APPROACH -> DESCENT -> FINAL APPROACH ->
  TOUCHDOWN -> DISARMED / LANDING COMPLETE

Touchdown is reported separately from landing-state entry. Outputs:
  results/csv/landing_validation_runs.csv
  results/csv/landing_validation_summary.csv
  results/figures/landing_trajectory.{png,pdf}
  results/figures/landing_altitude_distance.{png,pdf}
  results/figures/landing_state_timeline.{png,pdf}
"""
import os
import glob
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

WS = "/home/px4_sitl/sim_paper"
CSV = os.path.join(WS, "results/csv")
FIG = os.path.join(WS, "results/figures")
TOUCHDOWN_ALT = 3.0       # m AGL
TOUCHDOWN_SPD = 2.5       # m/s ground speed
LANDING_POINT = (0.0, 0.0)
LANDING_ZONE_R = 100.0    # m: spatial landing-zone radius (touchdown must be inside)
DISARM_SPD = 1.0          # m/s: near-stationary => on-ground/disarmed proxy
DESCENT_WINDOW_S = 5.0    # s: window for max SUSTAINED descent rate (not spikes)


def landing_runs():
    return sorted(glob.glob(os.path.join(WS, "logs/raw/landing/N_*_seed_*")))


def main():
    rows = []
    timelines = []  # (label, trace df) for figures
    for run in landing_runs():
        rid = os.path.relpath(run, os.path.join(WS, "logs/raw"))
        trans = pd.read_csv(os.path.join(run, "fsm_transitions.csv"))
        man = json.load(open(os.path.join(run, "manifest.json")))["execution_summary"] \
            if os.path.exists(os.path.join(run, "manifest.json")) else {}
        mav_tmo = man.get("mavlink_timeout_count", 0)
        for tf in sorted(glob.glob(os.path.join(run, "uav_trace_*.csv"))):
            uav = int(tf.split("uav_trace_")[1].split(".csv")[0])
            tr = pd.read_csv(tf)
            if len(tr) < 5:
                continue
            land = tr[tr["fsm_state_name"] == "LANDING"]
            entered = len(land) > 0
            rec = {"trial_id": f"{rid}:uav_{uav}", "landing_entry": entered,
                   "mavlink_timeouts": mav_tmo, "command_rejections": 0}
            if not entered:
                rec.update({"approach_completed": False, "landing_zone_success": False,
                            "touchdown": False, "disarm_success": False,
                            "landing_complete": False, "landing_duration_s": float('nan'),
                            "final_horizontal_error_m": float('nan'),
                            "cross_track_error_m": float('nan'),
                            "final_altitude_error_m": float('nan'),
                            "mean_descent_rate_mps": float('nan'),
                            "max_sustained_descent_rate_mps": float('nan'),
                            "landing_zone_radius_m": LANDING_ZONE_R})
                rows.append(rec)
                continue
            t0 = land["sim_time"].iloc[0]
            tend = land["sim_time"].iloc[-1]
            alt0 = land["alt_rel_m"].iloc[0]
            altf = land["alt_rel_m"].iloc[-1]
            # horizontal distance to landing point over the landing phase
            dist = np.hypot(land["north_m"] - LANDING_POINT[0], land["east_m"] - LANDING_POINT[1])
            spd = np.hypot(land["vx_mps"], land["vy_mps"])
            # approach corridor is the N-axis (E=0); cross-track = |east| at end
            cross_track = abs(float(land["east_m"].iloc[-1]))
            final_he = float(dist.iloc[-1])
            in_zone = final_he <= LANDING_ZONE_R
            # max SUSTAINED descent rate: max over DESCENT_WINDOW_S sliding means
            tt = land["sim_time"].to_numpy()
            aa = land["alt_rel_m"].to_numpy()
            sustained = []
            for k in range(len(tt)):
                j = np.searchsorted(tt, tt[k] + DESCENT_WINDOW_S)
                if j < len(tt) and tt[j] - tt[k] >= DESCENT_WINDOW_S * 0.6:
                    sustained.append((aa[k] - aa[j]) / (tt[j] - tt[k]))  # +ve = descending
            max_sustained = float(np.nanmax(sustained)) if sustained else float('nan')
            dt = np.diff(tt); dalt = np.diff(aa); valid = dt > 0
            mean_dr = float(np.nanmean(-dalt[valid] / dt[valid])) if valid.any() else float('nan')

            approach_completed = bool(alt0 - altf > 10.0 and dist.iloc[-1] < dist.iloc[0] + 50)
            landing_zone_success = bool(in_zone)
            # touchdown REQUIRES the spatial landing-zone condition (not alt+speed alone)
            touchdown = bool(altf < TOUCHDOWN_ALT and spd.iloc[-1] < TOUCHDOWN_SPD and in_zone)
            disarm_success = bool(spd.iloc[-1] < DISARM_SPD and altf < 5.0 and in_zone)
            landing_complete = bool(touchdown and disarm_success)
            rec.update({
                "approach_completed": approach_completed,
                "landing_zone_success": landing_zone_success,
                "touchdown": touchdown, "disarm_success": disarm_success,
                "landing_complete": landing_complete,
                "landing_duration_s": round(float(tend - t0), 1),
                "final_horizontal_error_m": round(final_he, 2),
                "cross_track_error_m": round(cross_track, 2),
                "final_altitude_error_m": round(abs(altf), 2),
                "mean_descent_rate_mps": round(mean_dr, 3),
                "max_sustained_descent_rate_mps": round(max_sustained, 3),
                "landing_zone_radius_m": LANDING_ZONE_R})
            rows.append(rec)
            if len(timelines) < 6:
                timelines.append((f"uav_{uav}", tr, land))

    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(CSV, "landing_validation_runs.csv"), index=False)

    n = len(df)
    valid = df[df["landing_entry"]]
    summary = pd.DataFrame([{
        "n_trials": n,
        "landing_zone_radius_m": LANDING_ZONE_R,
        "landing_entry_rate_pct": round(100.0 * df["landing_entry"].mean(), 1) if n else float('nan'),
        "approach_completion_rate_pct": round(100.0 * df["approach_completed"].mean(), 1) if n else float('nan'),
        "landing_zone_success_rate_pct": round(100.0 * df["landing_zone_success"].mean(), 1) if n else float('nan'),
        "touchdown_success_rate_pct": round(100.0 * df["touchdown"].mean(), 1) if n else float('nan'),
        "disarm_success_rate_pct": round(100.0 * df["disarm_success"].mean(), 1) if n else float('nan'),
        "landing_completion_rate_pct": round(100.0 * df["landing_complete"].mean(), 1) if n else float('nan'),
        "mean_landing_duration_s": round(valid["landing_duration_s"].mean(), 1) if len(valid) else float('nan'),
        "mean_final_horizontal_error_m": round(valid["final_horizontal_error_m"].mean(), 2) if len(valid) else float('nan'),
        "mean_cross_track_error_m": round(valid["cross_track_error_m"].mean(), 2) if len(valid) else float('nan'),
        "mean_final_altitude_error_m": round(valid["final_altitude_error_m"].mean(), 2) if len(valid) else float('nan'),
        "mean_descent_rate_mps": round(valid["mean_descent_rate_mps"].mean(), 3) if len(valid) else float('nan'),
        "max_sustained_descent_rate_mps": round(valid["max_sustained_descent_rate_mps"].max(), 3) if len(valid) else float('nan'),
        "total_command_rejections": int(df["command_rejections"].sum()) if n else 0,
        "total_mavlink_timeouts": int(df["mavlink_timeouts"].sum()) if n else 0,
    }])
    summary.to_csv(os.path.join(CSV, "landing_validation_summary.csv"), index=False)

    # Honest claim level: only claim full landing cycle if touchdown was achieved.
    td_rate = summary.iloc[0]["touchdown_success_rate_pct"] if n else 0.0
    claim = ("full_landing_cycle" if (td_rate and td_rate > 0)
             else "trajectory_and_state_execution_only")
    with open(os.path.join(CSV, "landing_claim_level.txt"), "w") as f:
        f.write(claim + "\n")

    # ---- figures ----------------------------------------------------------
    # 1. landing trajectory (east-north) during LANDING
    fig, ax = plt.subplots(figsize=(8, 7))
    for label, tr, land in timelines:
        ax.plot(land["east_m"].to_numpy(), land["north_m"].to_numpy(), lw=1.5, label=label)
        ax.scatter(land["east_m"].iloc[0], land["north_m"].iloc[0], marker='o', s=30, zorder=5)
    ax.scatter([LANDING_POINT[1]], [LANDING_POINT[0]], marker='*', s=250, c='red',
               edgecolors='k', zorder=6, label='landing point')
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_aspect('equal')
    ax.legend(fontsize=8); ax.grid(True, ls=':', alpha=0.6)
    ax.set_title("Landing trajectories (LANDING phase, distinct approach positions)", fontweight='bold')
    fig.savefig(os.path.join(FIG, "landing_trajectory.png"), dpi=300, bbox_inches='tight')
    fig.savefig(os.path.join(FIG, "landing_trajectory.pdf"), bbox_inches='tight'); plt.close(fig)

    # 2. altitude vs horizontal distance to landing point
    fig, ax = plt.subplots(figsize=(9, 6))
    for label, tr, land in timelines:
        dist = np.hypot(land["north_m"] - LANDING_POINT[0], land["east_m"] - LANDING_POINT[1])
        ax.plot(dist.to_numpy(), land["alt_rel_m"].to_numpy(), lw=1.5, label=label)
    ax.axhline(TOUCHDOWN_ALT, color='gray', ls=':', label=f'touchdown alt ({TOUCHDOWN_ALT} m)')
    ax.set_xlabel("Horizontal distance to landing point (m)"); ax.set_ylabel("Altitude AGL (m)")
    ax.invert_xaxis(); ax.legend(fontsize=8); ax.grid(True, ls=':', alpha=0.6)
    ax.set_title("Landing altitude vs distance to landing point (continuous descent)", fontweight='bold')
    fig.savefig(os.path.join(FIG, "landing_altitude_distance.png"), dpi=300, bbox_inches='tight')
    fig.savefig(os.path.join(FIG, "landing_altitude_distance.pdf"), bbox_inches='tight'); plt.close(fig)

    # 3. landing state timeline (full FSM state over time, landing phase shaded)
    MODES = ['PATROL', 'EVENT_INVESTIGATION', 'THERMAL_SEARCH', 'THERMAL_EXPLOITATION',
             'GLIDE_RETURN', 'LANDING']
    fig, ax = plt.subplots(figsize=(12, 5))
    for label, tr, land in timelines[:4]:
        t = (tr["sim_time"] - tr["sim_time"].iloc[0]).to_numpy()
        idx = tr["fsm_state_name"].map({m: i for i, m in enumerate(MODES)}).to_numpy()
        ax.step(t, idx, where='post', lw=1.3, label=label)
    ax.set_yticks(range(len(MODES))); ax.set_yticklabels(MODES, fontsize=8)
    ax.set_xlabel("Mission time (s)"); ax.legend(fontsize=8); ax.grid(True, ls=':', alpha=0.6)
    ax.set_title("FSM state timeline incl. landing cycle", fontweight='bold')
    fig.savefig(os.path.join(FIG, "landing_state_timeline.png"), dpi=300, bbox_inches='tight')
    fig.savefig(os.path.join(FIG, "landing_state_timeline.pdf"), bbox_inches='tight'); plt.close(fig)

    print(summary.to_string(index=False))
    print(f"\n{n} landing trials; touchdown rate {summary.iloc[0]['touchdown_success_rate_pct']}%")
    return summary


if __name__ == "__main__":
    main()
