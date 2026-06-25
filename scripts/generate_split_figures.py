#!/usr/bin/env python3
"""Generate individual single-column (IEEE) figures from existing CSV/log
outputs. Each former panel becomes its own figure file (PNG 300 dpi + vector
PDF), sized for one IEEE column (~3.4 in wide). No internal titles.
"""
import os
import glob
import json
import math
import numpy as np
import pandas as pd
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D

WS = "/home/px4_sitl/sim_paper"
SRC = os.path.join(WS, "results/csv")
RAW = os.path.join(WS, "logs/raw")
FIG = os.path.join(WS, "results/publication/figures")
os.makedirs(FIG, exist_ok=True)

UAV_COLORS = ['#1f77b4', '#2ca02c', '#9467bd', '#d62728', '#e377c2', '#17becf']
MODE_COLORS = {'PATROL': '#1f77b4', 'EVENT_INVESTIGATION': '#d62728',
               'THERMAL_SEARCH': '#bcbd22', 'THERMAL_EXPLOITATION': '#ff7f0e',
               'GLIDE_RETURN': '#2ca02c', 'LANDING': '#7f7f7f'}
MODES = list(MODE_COLORS.keys())
COL_W = 3.4  # IEEE single-column width (in)

plt.rcParams.update({
    'font.family': 'serif', 'font.size': 8, 'axes.labelsize': 8,
    'axes.titlesize': 8, 'xtick.labelsize': 7, 'ytick.labelsize': 7,
    'legend.fontsize': 6.5, 'lines.linewidth': 1.1, 'lines.markersize': 3.5,
    'axes.grid': True, 'grid.linestyle': ':', 'grid.alpha': 0.5,
    'figure.dpi': 100, 'savefig.bbox': 'tight'})


def rd(name):
    p = os.path.join(SRC, name)
    return pd.read_csv(p) if os.path.exists(p) else pd.DataFrame()


def newfig(h=2.5, w=COL_W):
    return plt.subplots(figsize=(w, h))


def savep(fig, name):
    fig.savefig(os.path.join(FIG, name + ".png"), dpi=300, bbox_inches='tight')
    fig.savefig(os.path.join(FIG, name + ".pdf"), bbox_inches='tight')
    plt.close(fig)
    print("  " + name)


def case_study_run():
    cands = sorted(glob.glob(os.path.join(RAW, "nominal/N_*_seed_*")))
    cands += sorted(glob.glob(os.path.join(RAW, "stochastic/N_*_seed_*")))
    best, bs = None, -1
    for d in cands:
        sp = os.path.join(d, "metrics_summary.json")
        if not os.path.exists(sp):
            continue
        s = json.load(open(sp))
        score = 10 * s.get('_fleet', {}).get('hp_investigated_count', 0) + \
            sum(u.get('thermal_encounters', 0) for k, u in s.items() if k.startswith('uav'))
        if 'nominal' in d:
            score += 100
        if score > bs:
            best, bs = d, score
    return best


def load_run(run):
    traces = {int(p.split('uav_trace_')[1].split('.csv')[0]): pd.read_csv(p)
              for p in sorted(glob.glob(os.path.join(run, "uav_trace_*.csv")))}
    trans = pd.read_csv(os.path.join(run, "fsm_transitions.csv"))
    thermals = pd.read_csv(os.path.join(run, "thermal_field.csv"))
    events = pd.read_csv(os.path.join(run, "events.csv"))
    cfg = yaml.safe_load(open(os.path.join(run, "config.yaml")))
    return traces, trans, thermals, events, cfg


def planned_routes(cfg, n):
    fsm = cfg['fsm_node']['ros__parameters']
    out = {}
    for i in range(1, n + 1):
        k = f'patrol_waypoints_{i}' if f'patrol_waypoints_{i}' in fsm else 'patrol_waypoints_1'
        flat = fsm[k]
        out[i] = [(flat[j], flat[j + 1]) for j in range(0, len(flat), 3)]
    return out


# ============================================================ trajectories
def trajectories(run):
    traces, trans, thermals, events, cfg = load_run(run)
    routes = planned_routes(cfg, len(traces))
    # planned route
    fig, ax = newfig(2.9)
    for i, r in routes.items():
        ax.plot([p[1] for p in r], [p[0] for p in r], '-o', color=UAV_COLORS[(i - 1) % 6],
                lw=1.0, ms=2, label=f'UAV {i}')
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_aspect('equal')
    ax.legend(ncol=2, fontsize=5.5)
    savep(fig, "planned_route")
    # executed
    fig, ax = newfig(3.0)
    for i, r in routes.items():
        ax.plot([p[1] for p in r], [p[0] for p in r], '--', color='gray', lw=0.5, alpha=0.6, zorder=1)
    for tid, g in thermals[thermals['active'] == 1].groupby('thermal_id'):
        ax.add_patch(patches.Circle((g['center_east_m'].median(), g['center_north_m'].median()),
                     g['radius_m'].iloc[0], facecolor='orange', alpha=0.12, edgecolor='#d97706',
                     lw=0.4, zorder=2))
    hp = events[events['is_high_priority'] == 1]
    inv = hp[hp['final_state'] == 'investigated']
    ax.scatter(hp['east_m'], hp['north_m'], marker='*', s=30, c='#999', edgecolors='k', linewidths=0.3, zorder=6)
    ax.scatter(inv['east_m'], inv['north_m'], marker='*', s=55, c='#d62728', edgecolors='k', linewidths=0.4, zorder=7)
    for i, tr in traces.items():
        c = UAV_COLORS[(i - 1) % 6]
        ax.plot(tr['east_m'].to_numpy(), tr['north_m'].to_numpy(), color=c, lw=0.6, alpha=0.8, zorder=3)
        for mode, st in [('THERMAL_EXPLOITATION', dict(color='#ff7f0e', lw=1.5)),
                         ('EVENT_INVESTIGATION', dict(color='#d62728', lw=1.5)),
                         ('GLIDE_RETURN', dict(color='#2ca02c', lw=1.0)),
                         ('LANDING', dict(color='#7f7f7f', lw=1.0))]:
            seg = tr[tr['fsm_state_name'] == mode]
            if len(seg) > 1:
                for b in np.split(seg.index.values, np.where(np.diff(seg.index.values) > 1)[0] + 1):
                    if len(b) > 1:
                        ax.plot(tr.loc[b, 'east_m'].to_numpy(), tr.loc[b, 'north_m'].to_numpy(), zorder=5, **st)
        ax.scatter(tr['east_m'].iloc[0], tr['north_m'].iloc[0], marker='o', s=10, c=c, edgecolors='k', zorder=8)
        ax.scatter(tr['east_m'].iloc[-1], tr['north_m'].iloc[-1], marker='X', s=16, c=c, edgecolors='k', zorder=8)
    handles = [Line2D([], [], color='gray', ls='--', lw=0.8, label='Planned'),
               Line2D([], [], color='#ff7f0e', lw=1.5, label='Thermal expl.'),
               Line2D([], [], color='#d62728', lw=1.5, label='Event loiter'),
               Line2D([], [], color='#2ca02c', lw=1.0, label='Glide'),
               Line2D([], [], color='#7f7f7f', lw=1.0, label='Landing'),
               Line2D([], [], marker='*', color='w', mfc='#d62728', mec='k', ms=8, label='HP (inv.)'),
               patches.Patch(facecolor='orange', alpha=0.2, ec='#d97706', label='Thermal')]
    ax.legend(handles=handles, fontsize=5, ncol=2, loc='upper left')
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_aspect('equal')
    savep(fig, "executed_trajectories")


# ============================================================ px4-jsbsim
def px4jsbsim():
    raw = pd.read_csv(os.path.join(RAW, "verification/px4jsbsim_raw.csv"))
    INIT = 20.0
    uav = sorted(raw['uav_id'].unique())[0]
    g = raw[raw['uav_id'] == uav].sort_values('sim_time')
    t = (g['sim_time'] - g['sim_time'].iloc[0]).to_numpy()

    def ae(a, b):
        d = a - b
        return (d + np.pi) % (2 * np.pi) - np.pi
    # altitude
    fig, ax = newfig(2.2)
    ax.plot(t, (-g.est_z).to_numpy(), color='#1f77b4', label='PX4 EKF')
    ax.plot(t, (-g.gt_z).to_numpy(), color='#d62728', ls='--', label='JSBSim FDM')
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Altitude (m)"); ax.legend()
    savep(fig, "pxjsbsim_altitude")
    fig, ax = newfig(2.2)
    ax.plot(t, np.abs((-g.est_z) - (-g.gt_z)).to_numpy(), color='k')
    ax.axvspan(0, INIT, color='orange', alpha=0.15, label='init (excl.)')
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Altitude error (m)"); ax.legend()
    savep(fig, "pxjsbsim_altitude_error")
    es = np.sqrt(g.est_vx**2 + g.est_vy**2 + g.est_vz**2).to_numpy()
    gs = np.sqrt(g.gt_vx**2 + g.gt_vy**2 + g.gt_vz**2).to_numpy()
    fig, ax = newfig(2.2)
    ax.plot(t, es, color='#1f77b4', label='PX4 EKF'); ax.plot(t, gs, color='#d62728', ls='--', label='JSBSim FDM')
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Ground speed (m/s)"); ax.legend()
    savep(fig, "pxjsbsim_speed")
    fig, ax = newfig(2.2)
    ax.plot(t, np.abs(es - gs), color='k'); ax.axvspan(0, INIT, color='orange', alpha=0.15)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Speed error (m/s)")
    savep(fig, "pxjsbsim_speed_error")
    fig, ax = newfig(2.2)
    ax.plot(t, np.degrees(g.est_pitch).to_numpy(), color='#1f77b4', label='PX4 EKF')
    ax.plot(t, np.degrees(g.gt_pitch).to_numpy(), color='#d62728', ls='--', label='JSBSim FDM')
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Pitch (deg)"); ax.legend()
    savep(fig, "pxjsbsim_attitude")
    att = np.degrees(np.sqrt(ae(g.est_roll, g.gt_roll)**2 + ae(g.est_pitch, g.gt_pitch)**2 + ae(g.est_yaw, g.gt_yaw)**2)).to_numpy()
    fig, ax = newfig(2.2)
    ax.plot(t, att, color='k'); ax.axvspan(0, INIT, color='orange', alpha=0.15, label='init (excl.)')
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Attitude error (deg)"); ax.legend()
    savep(fig, "pxjsbsim_attitude_error")


# ============================================================ baselines
def baselines():
    d = rd("reduced_framework_baselines.csv")
    order = ["full_framework", "no_thermal", "no_event_response", "non_energy_aware_fsm",
             "simplified_battery", "coverage_only"]
    d = d.set_index("configuration").reindex(order).reset_index()
    lab = [c.replace("_", "\n") for c in d["configuration"]]
    xs = np.arange(len(d))
    for m, yl, nm in [("final_soc_pct", "Final SOC (%)", "baseline_final_soc"),
                      ("propulsion_energy_wh", "Propulsion energy (Wh)", "baseline_energy"),
                      ("hp_investigated_pct", "HP investigated (%)", "baseline_hp_investigated"),
                      ("thermalling_time_s", "Thermalling duration (s)", "baseline_thermalling")]:
        fig, ax = newfig(2.6)
        ax.bar(xs, d[m + "_mean"].to_numpy(), yerr=d[m + "_std"].fillna(0).to_numpy(),
               capsize=2, color='#4477aa', edgecolor='k', alpha=0.85)
        ax.set_xticks(xs); ax.set_xticklabels(lab, fontsize=5, rotation=0); ax.set_ylabel(yl)
        savep(fig, nm)


# ============================================================ energy
def energy(run):
    eb = rd("energy_budget_by_mode.csv"); eb = eb[eb["mode"].isin(MODES)]
    fig, ax = newfig(2.4)
    ax.barh(range(len(eb)), eb["propulsion_energy_wh_mean"].to_numpy(),
            xerr=eb["propulsion_energy_wh_std"].fillna(0).to_numpy(), capsize=2,
            color=[MODE_COLORS[m] for m in eb["mode"]], edgecolor='k', alpha=0.85)
    ax.set_yticks(range(len(eb))); ax.set_yticklabels(eb["mode"], fontsize=6)
    ax.set_xlabel("Propulsion energy (Wh)")
    savep(fig, "energy_by_mode")
    traces, _, _, _, _ = load_run(run)
    fig, ax = newfig(2.5)
    for i, tr in traces.items():
        t = (tr['sim_time'] - tr['sim_time'].iloc[0]).to_numpy()
        soc = pd.to_numeric(tr['soc_pct'], errors='coerce').to_numpy()
        ax.plot(t, soc, color='lightgray', lw=0.4, zorder=1)
        for m in MODES:
            sel = (tr['fsm_state_name'] == m).to_numpy()
            ax.scatter(t[sel], soc[sel], s=0.8, color=MODE_COLORS[m], zorder=2)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("SOC (%)")
    ax.legend(handles=[Line2D([], [], marker='o', ls='', color=MODE_COLORS[m], ms=3, label=m) for m in MODES],
              fontsize=5, loc='upper right')
    savep(fig, "soc_by_mode")
    bat = rd("battery_model_comparison.csv"); bat = bat[bat["model"] != "external_realflight_reference"]
    fig, ax = newfig(2.5)
    xs = np.arange(len(bat))
    ax.bar(xs, bat["final_soc_pct_mean"].to_numpy(), yerr=bat["final_soc_pct_std"].fillna(0).to_numpy(),
           capsize=2, color=['#2ca02c', '#ff7f0e', '#7f7f7f'][:len(bat)], edgecolor='k', alpha=0.85)
    ax.set_xticks(xs); ax.set_xticklabels(["online\nestimator", "constant\npower", "PX4\nestimate"][:len(bat)], fontsize=6)
    ax.set_ylabel("Final SOC (%)")
    savep(fig, "battery_model_comparison")


# ============================================================ thermal sens
def thermal_sens():
    d = rd("thermal_sensitivity.csv").set_index("level").reindex(["low", "nominal", "high"]).reset_index()
    xs = np.arange(len(d))
    for m, yl, nm in [("thermal_encounters_per_uav", "Thermal encounters per UAV", "thermal_encounters"),
                      ("thermalling_time_s", "Thermalling duration (s)", "thermal_duration"),
                      ("exploitation_saving_wh", "Energy saving vs cruise (Wh)", "thermal_saving"),
                      ("final_soc_pct", "Final SOC (%)", "thermal_final_soc")]:
        fig, ax = newfig(2.4)
        sc = m + "_std"
        yerr = d[sc].fillna(0).to_numpy() if sc in d else None
        ax.errorbar(xs, d[m + "_mean"].to_numpy(), yerr=yerr, fmt='-o', color='#ee6677', capsize=3)
        ax.set_xticks(xs); ax.set_xticklabels(d["level"]); ax.set_xlabel("Thermal condition"); ax.set_ylabel(yl)
        savep(fig, nm)


# ============================================================ thermal trace
def thermal_trace(run):
    traces, trans, thermals, events, cfg = load_run(run)
    s = json.load(open(os.path.join(run, "metrics_summary.json")))
    best = None
    for k, u in s.items():
        if not k.startswith('uav_'):
            continue
        for seg in u.get('thermal_segments', []):
            if best is None or seg['duration_s'] > best[1]['duration_s']:
                best = (int(k.split('_')[1]), seg)
    if not best:
        return
    uav, seg = best
    tr = traces[uav]
    segp = tr[(tr['sim_time'] >= seg['entry_t']) & (tr['sim_time'] <= seg['exit_t'])]
    win = tr[(tr['sim_time'] >= seg['entry_t'] - 20) & (tr['sim_time'] <= seg['exit_t'] + 20)]
    fig, ax = newfig(2.7)
    for tid, g in thermals[thermals['active'] == 1].groupby('thermal_id'):
        ce, cn = g['center_east_m'].median(), g['center_north_m'].median()
        if abs(ce - segp['east_m'].mean()) < 400:
            ax.add_patch(patches.Circle((ce, cn), g['radius_m'].iloc[0], facecolor='orange', alpha=0.2, edgecolor='#d97706'))
    ax.plot(win['east_m'].to_numpy(), win['north_m'].to_numpy(), color='gray', lw=0.7)
    ax.plot(segp['east_m'].to_numpy(), segp['north_m'].to_numpy(), color='#ff7f0e', lw=1.6)
    ax.scatter(segp['east_m'].iloc[0], segp['north_m'].iloc[0], marker='^', s=40, c='#ff7f0e', ec='k', zorder=5)
    ax.scatter(segp['east_m'].iloc[-1], segp['north_m'].iloc[-1], marker='v', s=40, c='#1f77b4', ec='k', zorder=5)
    if len(segp):
        cx, cy = segp['east_m'].mean(), segp['north_m'].mean()
        sp = max(150, segp['east_m'].std() * 3, segp['north_m'].std() * 3)
        ax.set_xlim(cx - sp, cx + sp); ax.set_ylim(cy - sp, cy + sp)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_aspect('equal')
    ax.legend(handles=[Line2D([], [], marker='^', color='w', mfc='#ff7f0e', mec='k', ms=6, label='entry'),
                       Line2D([], [], marker='v', color='w', mfc='#1f77b4', mec='k', ms=6, label='exit')], fontsize=6)
    savep(fig, "thermal_trace_path")
    t = (win['sim_time'] - win['sim_time'].iloc[0]).to_numpy(); e0 = win['sim_time'].iloc[0]
    fig, ax = newfig(2.5)
    ax.plot(t, win['alt_rel_m'].to_numpy(), color='#1f77b4', label='Altitude (m)')
    ax.axvspan(seg['entry_t'] - e0, seg['exit_t'] - e0, color='#ff7f0e', alpha=0.15)
    ax.set_xlabel("Time (s)"); ax.set_ylabel("Altitude (m)")
    ax2 = ax.twinx()
    ax2.plot(t, pd.to_numeric(win['energy_consumed_wh'], errors='coerce').to_numpy(), color='#d62728', lw=1.0, label='Energy (Wh)')
    ax2.plot(t, pd.to_numeric(win['wind_w_mps'], errors='coerce').to_numpy(), color='#2ca02c', lw=0.7, label='Updraft (m/s)')
    ax2.set_ylabel("Energy (Wh) / updraft (m/s)"); ax2.grid(False)
    l1, la1 = ax.get_legend_handles_labels(); l2, la2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, la1 + la2, fontsize=5.5, loc='center right')
    savep(fig, "thermal_trace_time")


# ============================================================ events
def events_fig():
    d = rd("event_sensitivity.csv").set_index("level").reindex(["low", "nominal", "high"]).reset_index()
    xs = np.arange(len(d)); w = 0.25
    fig, ax = newfig(2.7)
    ax.bar(xs - w, d["hp_investigated_count_mean"].to_numpy(), w, label='Investigated', color='#2ca02c', ec='k')
    ax.bar(xs, d["hp_expired_count_mean"].to_numpy(), w, label='Expired', color='#d62728', ec='k')
    ax.bar(xs + w, d["hp_unresolved_count_mean"].to_numpy(), w, label='Unresolved', color='#7f7f7f', ec='k')
    ax.set_xticks(xs); ax.set_xticklabels(d["level"]); ax.set_xlabel("Event load")
    ax.set_ylabel("Mean HP events / run")
    ax2 = ax.twinx()
    ax2.plot(xs, d["hp_detected_pct_mean"].to_numpy(), '-o', color='#1f77b4', label='Detected (%)')
    ax2.plot(xs, d["hp_investigated_pct_mean"].to_numpy(), '-s', color='#9467bd', label='Investigated (%)')
    ax2.set_ylabel("Percentage (%)"); ax2.grid(False)
    l1, la1 = ax.get_legend_handles_labels(); l2, la2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, la1 + la2, fontsize=5, loc='upper left')
    savep(fig, "event_outcomes")
    # latency with n
    level_runs = {"low": sorted(glob.glob(os.path.join(RAW, "event_sensitivity/low/N_6_seed_*"))),
                  "nominal": [x for x in sorted(glob.glob(os.path.join(RAW, "stochastic/N_6_seed_*")))
                              if int(x.split("seed_")[1]) in (42, 43, 44, 45, 46)],
                  "high": sorted(glob.glob(os.path.join(RAW, "event_sensitivity/high/N_6_seed_*")))}
    fig, ax = newfig(2.5)
    for xi, lvl in enumerate(d["level"]):
        lats = []
        for run in level_runs.get(lvl, []):
            ep = os.path.join(run, "events.csv")
            if not os.path.exists(ep):
                continue
            ev = pd.read_csv(ep)
            inv = ev[(ev["is_high_priority"] == 1) & (ev["final_state"] == "investigated")]
            for _, e in inv.iterrows():
                try:
                    fl = float(e["first_detect_time"]); st = float(e["investigation_start_time"])
                    if not (math.isnan(fl) or math.isnan(st)) and st >= fl:
                        lats.append(st - fl)
                except (ValueError, TypeError):
                    pass
        nlat = len(lats)
        if nlat >= 2:
            m, sd, med = float(np.mean(lats)), float(np.std(lats, ddof=1)), float(np.median(lats))
            ax.errorbar([xi], [m], yerr=[[min(sd, m)], [sd]], fmt='o', color='#4477aa', capsize=4, ms=5)
            ax.scatter([xi], [med], marker='_', s=120, color='#d62728', zorder=5)
        elif nlat == 1:
            ax.scatter([xi], [lats[0]], marker='o', s=30, color='#4477aa', zorder=5)
        ax.annotate(f"n={nlat}", (xi, 0), textcoords="offset points", xytext=(0, 3), fontsize=6, ha='center')
    ax.set_ylim(bottom=0); ax.set_xticks(xs); ax.set_xticklabels(d["level"]); ax.set_xlabel("Event load")
    ax.set_ylabel("HP investigation latency (s)")
    ax.legend(handles=[Line2D([], [], marker='o', ls='', color='#4477aa', label='mean ($\\pm$SD, $n\\geq2$)'),
                       Line2D([], [], marker='_', ls='', color='#d62728', label='median')], fontsize=5.5)
    savep(fig, "event_latency")


# ============================================================ stochastic
def stochastic():
    d = rd("stochastic_runs_per_seed.csv")
    for m, yl, nm in [("route_completion_pct", "Route completion (%)", "stoch_route"),
                      ("final_soc_pct", "Final SOC (%)", "stoch_soc"),
                      ("thermal_encounters", "Thermal encounters", "stoch_encounters"),
                      ("thermalling_time_s", "Thermalling duration (s)", "stoch_thermalling"),
                      ("hp_investigated_pct", "HP investigated (%)", "stoch_hp_inv"),
                      ("all_detected_pct", "Detected events (%)", "stoch_detected")]:
        vals = pd.to_numeric(d[m], errors='coerce').dropna().to_numpy()
        fig, ax = newfig(2.3, w=2.4)
        bp = ax.boxplot([vals], widths=0.5, showmeans=True, patch_artist=True,
                        meanprops=dict(marker='D', mfc='#d62728', mec='k', ms=4))
        for b in bp['boxes']:
            b.set(facecolor='#aac7e2', alpha=0.8)
        ax.set_xticks([1]); ax.set_xticklabels([f"R={len(vals)}"], fontsize=6); ax.set_ylabel(yl)
        savep(fig, nm)


# ============================================================ scalability
def scalability():
    d = rd("scalability_overhead.csv")
    xs = d["fleet_size_n"].to_numpy()
    for m, s, yl, nm in [("cpu_pct_mean", "cpu_pct_std", "CPU usage (%)", "scal_cpu"),
                         ("mem_pct_mean", "mem_pct_std", "Memory usage (%)", "scal_mem"),
                         ("rtf_mean", "rtf_std", "Real-time factor", "scal_rtf"),
                         ("ros2_latency_ms_mean", "ros2_latency_ms_std", "ROS 2 latency (ms)", "scal_latency")]:
        fig, ax = newfig(2.3)
        yerr = d[s].fillna(0).to_numpy() if s in d else None
        ax.errorbar(xs, d[m].to_numpy(), yerr=yerr, fmt='-o', color='#228833', capsize=3)
        if m == 'rtf_mean':
            ax.axhline(1.0, color='gray', ls=':')
        ax.set_xticks(xs); ax.set_xlabel("Fleet size (UAVs)"); ax.set_ylabel(yl)
        savep(fig, nm)


# ============================================================ landing
def landing():
    runs = sorted(glob.glob(os.path.join(RAW, "landing/N_*_seed_*")))
    lr = rd("landing_validation_runs.csv")
    R = float(lr["landing_zone_radius_m"].iloc[0]) if "landing_zone_radius_m" in lr else 100.0
    traces = {}
    for run in runs:
        for tf in sorted(glob.glob(os.path.join(run, "uav_trace_*.csv"))):
            u = int(tf.split('uav_trace_')[1].split('.csv')[0])
            traces[u] = pd.read_csv(tf)
    td = {int(r.trial_id.split('uav_')[1]): bool(r.touchdown) for _, r in lr.iterrows() if 'uav_' in str(r.trial_id)}
    dis = {int(r.trial_id.split('uav_')[1]): bool(r.disarm_success) for _, r in lr.iterrows() if 'uav_' in str(r.trial_id)}
    # tracks
    fig, ax = newfig(2.9)
    ax.add_patch(patches.Circle((0, 0), R, fill=False, ls='--', ec='#d62728', lw=1.0))
    for u, tr in traces.items():
        land = tr[tr['fsm_state_name'] == 'LANDING']
        if len(land) == 0:
            continue
        c = '#228833' if td.get(u) else '#cc3311'
        ax.plot(land['east_m'].to_numpy(), land['north_m'].to_numpy(), color=c, lw=0.9)
        ax.scatter(land['east_m'].iloc[-1], land['north_m'].iloc[-1], marker=('o' if td.get(u) else 'x'), s=25, c=c, zorder=5)
    ax.scatter([0], [0], marker='*', s=90, c='k', zorder=6)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_aspect('equal')
    ax.legend(handles=[Line2D([], [], color='#228833', lw=1.2, label='touchdown ok'),
                       Line2D([], [], color='#cc3311', lw=1.2, label='failed'),
                       Line2D([], [], color='#d62728', ls='--', label=f'zone R={R:.0f} m')], fontsize=5.5)
    savep(fig, "landing_tracks")
    # alt vs distance
    fig, ax = newfig(2.4)
    for u, tr in traces.items():
        land = tr[tr['fsm_state_name'] == 'LANDING']
        if len(land) == 0:
            continue
        dist = np.hypot(land['north_m'], land['east_m']).to_numpy()
        c = '#228833' if td.get(u) else '#cc3311'
        ax.plot(dist, land['alt_rel_m'].to_numpy(), color=c, lw=0.9)
    ax.axvline(R, color='#d62728', ls='--', lw=0.9); ax.axhline(3.0, color='gray', ls=':', lw=0.7)
    ax.invert_xaxis(); ax.set_xlabel("Distance to landing point (m)"); ax.set_ylabel("Altitude (m)")
    savep(fig, "landing_alt_distance")
    # subphase timeline
    FA = 15.0; TD = 3.0
    us = sorted(traces.keys())
    fig, ax = newfig(2.7)
    for row, u in enumerate(us):
        land = traces[u][traces[u]['fsm_state_name'] == 'LANDING']
        if len(land) == 0:
            continue
        t = (land['sim_time'] - land['sim_time'].iloc[0]).to_numpy()
        alt = land['alt_rel_m'].to_numpy()
        fa_idx = np.argmax(alt <= FA) if (alt <= FA).any() else None
        tdi = np.argmax(alt <= TD) if (alt <= TD).any() else None
        end_t = t[-1]; fa_t = t[fa_idx] if fa_idx is not None else end_t
        ax.plot([0, fa_t], [row, row], color='#4477aa', lw=3.5, solid_capstyle='butt')
        ax.plot([fa_t, end_t], [row, row], color='#ccbb44', lw=3.5, solid_capstyle='butt')
        ax.scatter([0], [row], marker='|', s=60, c='k', zorder=5)
        if tdi is not None and td.get(u):
            ax.scatter([t[tdi]], [row], marker='v', s=30, c='#228833', edgecolors='k', zorder=6)
        else:
            ax.scatter([end_t], [row], marker='x', s=30, c='#cc3311', zorder=6)
        ax.scatter([end_t], [row], marker=('s' if dis.get(u) else 'D'), s=18,
                   c=('#228833' if dis.get(u) else '#cc3311'), edgecolors='k', zorder=7)
    ax.set_yticks(range(len(us))); ax.set_yticklabels([f"UAV {u}" for u in us], fontsize=6)
    ax.set_xlabel("Time since landing entry (s)")
    ax.legend(handles=[Line2D([], [], color='#4477aa', lw=3, label='approach/descent'),
                       Line2D([], [], color='#ccbb44', lw=3, label='final approach'),
                       Line2D([], [], marker='v', ls='', mfc='#228833', mec='k', label='touchdown'),
                       Line2D([], [], marker='x', ls='', color='#cc3311', label='td failed'),
                       Line2D([], [], marker='s', ls='', mfc='#228833', mec='k', label='disarm'),
                       Line2D([], [], marker='D', ls='', mfc='#cc3311', mec='k', label='no disarm')],
              fontsize=4.8, ncol=2, loc='lower right')
    savep(fig, "landing_timeline")


def main():
    run = case_study_run()
    print("Split figures (case study:", os.path.relpath(run, RAW), ")")
    trajectories(run)
    px4jsbsim()
    baselines()
    energy(run)
    thermal_sens()
    thermal_trace(run)
    events_fig()
    stochastic()
    scalability()
    landing()
    print("done:", len(glob.glob(os.path.join(FIG, "*.png"))), "PNG figures")


if __name__ == "__main__":
    main()
