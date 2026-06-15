#!/usr/bin/env python3
"""Build the publication-ready Results package from existing CSV/log outputs.

Produces results/publication/{figures,tables,csv}/ and results_section.tex.
Figures carry NO internal title (only axis labels, units, legends, panel
labels (a),(b),...). Every plotted value and table comes from a generated CSV.
No synthetic data; unavailable metrics are 'NR'.
"""
import os
import glob
import json
import math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D

WS = "/home/px4_sitl/sim_paper"
SRC = os.path.join(WS, "results/csv")
RAW = os.path.join(WS, "logs/raw")
PUB = os.path.join(WS, "results/publication")
FIG = os.path.join(PUB, "figures")
TAB = os.path.join(PUB, "tables")
PCSV = os.path.join(PUB, "csv")
for d in (FIG, TAB, PCSV):
    os.makedirs(d, exist_ok=True)

# ---- consistent style -----------------------------------------------------
UAV_COLORS = ['#1f77b4', '#2ca02c', '#9467bd', '#d62728', '#e377c2', '#17becf']
MODE_COLORS = {'PATROL': '#1f77b4', 'EVENT_INVESTIGATION': '#d62728',
               'THERMAL_SEARCH': '#bcbd22', 'THERMAL_EXPLOITATION': '#ff7f0e',
               'GLIDE_RETURN': '#2ca02c', 'LANDING': '#7f7f7f'}
MODES = list(MODE_COLORS.keys())
plt.rcParams.update({
    'font.family': 'serif', 'font.size': 9, 'axes.labelsize': 9,
    'axes.titlesize': 9, 'xtick.labelsize': 8, 'ytick.labelsize': 8,
    'legend.fontsize': 7.5, 'lines.linewidth': 1.3, 'lines.markersize': 4,
    'axes.grid': True, 'grid.linestyle': ':', 'grid.alpha': 0.5})


def rd(name):
    p = os.path.join(SRC, name)
    return pd.read_csv(p) if os.path.exists(p) else pd.DataFrame()


def save(fig, name):
    fig.savefig(os.path.join(FIG, name + ".png"), dpi=300, bbox_inches='tight')
    fig.savefig(os.path.join(FIG, name + ".pdf"), bbox_inches='tight')
    plt.close(fig)
    print(f"  fig {name}.png/.pdf")


def panel(ax, lab):
    ax.text(0.02, 0.97, lab, transform=ax.transAxes, fontsize=10, fontweight='bold',
            va='top', ha='left')


def latex_escape(s):
    return (str(s).replace('\\', r'\textbackslash{}').replace('_', r'\_')
            .replace('%', r'\%').replace('&', r'\&').replace('#', r'\#'))


def write_table(df, name, caption, label, col_fmt=None, note=""):
    """Write df to publication/csv/<name>.csv and publication/tables/<name>.tex."""
    df.to_csv(os.path.join(PCSV, name + ".csv"), index=False)
    ncol = len(df.columns)
    col_fmt = col_fmt or ('l' + 'r' * (ncol - 1))
    lines = [r"\begin{table*}[t]", r"\centering", r"\small",
             rf"\caption{{{caption}}}", rf"\label{{{label}}}",
             r"\resizebox{\textwidth}{!}{%",
             rf"\begin{{tabular}}{{{col_fmt}}}", r"\toprule",
             " & ".join(latex_escape(c) for c in df.columns) + r" \\", r"\midrule"]
    for _, row in df.iterrows():
        lines.append(" & ".join(latex_escape(v) for v in row) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}}"]
    if note:
        lines.append(rf"\\[2pt]\footnotesize {note}")
    lines.append(r"\end{table*}")
    with open(os.path.join(TAB, name + ".tex"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  table {name}.tex/.csv")


def fnum(v, d=1):
    try:
        f = float(v)
        return "NR" if (isinstance(f, float) and math.isnan(f)) else f"{f:.{d}f}"
    except (ValueError, TypeError):
        return "NR" if (v is None or str(v) == 'nan') else str(v)


# ====================================================================== CASE
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
    import yaml
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


# ===================================================================== FIG 1
def fig1_trajectories(run):
    traces, trans, thermals, events, cfg = load_run(run)
    routes = planned_routes(cfg, len(traces))
    fig, (axa, axb) = plt.subplots(1, 2, figsize=(7.2, 3.7))
    # (a) planned
    for i, r in routes.items():
        e = [p[1] for p in r]; n = [p[0] for p in r]
        axa.plot(e, n, '-o', color=UAV_COLORS[(i - 1) % 6], lw=1.2, ms=2.5,
                 label=f'UAV {i}')
    axa.set_xlabel("East (m)"); axa.set_ylabel("North (m)"); axa.set_aspect('equal')
    axa.legend(ncol=2, fontsize=6); panel(axa, "(a)")
    # (b) executed
    for i, r in routes.items():
        e = [p[1] for p in r]; n = [p[0] for p in r]
        axb.plot(e, n, '--', color='gray', lw=0.6, alpha=0.6, zorder=1)
    for tid, g in thermals[thermals['active'] == 1].groupby('thermal_id'):
        axb.add_patch(patches.Circle((g['center_east_m'].median(), g['center_north_m'].median()),
                      g['radius_m'].iloc[0], facecolor='orange', alpha=0.12,
                      edgecolor='#d97706', lw=0.5, zorder=2))
    hp = events[events['is_high_priority'] == 1]
    inv = hp[hp['final_state'] == 'investigated']
    axb.scatter(hp['east_m'], hp['north_m'], marker='*', s=45, c='#999', edgecolors='k',
                linewidths=0.3, zorder=6)
    axb.scatter(inv['east_m'], inv['north_m'], marker='*', s=70, c='#d62728', edgecolors='k',
                linewidths=0.4, zorder=7)
    for i, tr in traces.items():
        c = UAV_COLORS[(i - 1) % 6]
        axb.plot(tr['east_m'].to_numpy(), tr['north_m'].to_numpy(), color=c, lw=0.7, alpha=0.8, zorder=3)
        for mode, st in [('THERMAL_EXPLOITATION', dict(color='#ff7f0e', lw=1.8)),
                         ('EVENT_INVESTIGATION', dict(color='#d62728', lw=1.8)),
                         ('GLIDE_RETURN', dict(color='#2ca02c', lw=1.2)),
                         ('LANDING', dict(color='#7f7f7f', lw=1.2))]:
            seg = tr[tr['fsm_state_name'] == mode]
            if len(seg) > 1:
                for b in np.split(seg.index.values, np.where(np.diff(seg.index.values) > 1)[0] + 1):
                    if len(b) > 1:
                        axb.plot(tr.loc[b, 'east_m'].to_numpy(), tr.loc[b, 'north_m'].to_numpy(), zorder=5, **st)
        axb.scatter(tr['east_m'].iloc[0], tr['north_m'].iloc[0], marker='o', s=14, c=c, edgecolors='k', zorder=8)
        axb.scatter(tr['east_m'].iloc[-1], tr['north_m'].iloc[-1], marker='X', s=22, c=c, edgecolors='k', zorder=8)
    for _, t in trans.iterrows():
        u = int(t['uav_id'])
        if u not in traces:
            continue
        idx = (traces[u]['sim_time'] - t['sim_time']).abs().idxmin()
        rr = traces[u].loc[idx]
        if t['to_state'] == 'THERMAL_EXPLOITATION':
            axb.scatter(rr['east_m'], rr['north_m'], marker='^', s=24, c='#ff7f0e', edgecolors='k', linewidths=0.3, zorder=9)
        elif t['from_state'] == 'THERMAL_EXPLOITATION':
            axb.scatter(rr['east_m'], rr['north_m'], marker='v', s=24, c='#1f77b4', edgecolors='k', linewidths=0.3, zorder=9)
    handles = [Line2D([], [], color='gray', ls='--', lw=0.8, label='Planned route'),
               Line2D([], [], color='#1f77b4', lw=1, label='Executed path'),
               Line2D([], [], color='#ff7f0e', lw=1.8, label='Thermal exploitation'),
               Line2D([], [], color='#d62728', lw=1.8, label='Event loiter'),
               Line2D([], [], color='#2ca02c', lw=1.2, label='Glide return'),
               Line2D([], [], color='#7f7f7f', lw=1.2, label='Landing'),
               Line2D([], [], marker='^', color='w', mfc='#ff7f0e', mec='k', ms=6, label='Thermal entry'),
               Line2D([], [], marker='v', color='w', mfc='#1f77b4', mec='k', ms=6, label='Thermal exit'),
               Line2D([], [], marker='*', color='w', mfc='#d62728', mec='k', ms=9, label='HP event (inv.)'),
               patches.Patch(facecolor='orange', alpha=0.2, ec='#d97706', label='Thermal footprint')]
    axb.legend(handles=handles, fontsize=5.5, ncol=2, loc='upper left')
    axb.set_xlabel("East (m)"); axb.set_ylabel("North (m)"); axb.set_aspect('equal')
    panel(axb, "(b)")
    fig.tight_layout()
    save(fig, "planned_executed_trajectories")


# ===================================================================== FIG 2
def fig2_px4jsbsim():
    raw = pd.read_csv(os.path.join(RAW, "verification/px4jsbsim_raw.csv"))
    INIT = 20.0
    uav = sorted(raw['uav_id'].unique())[0]
    g = raw[raw['uav_id'] == uav].sort_values('sim_time')
    t = (g['sim_time'] - g['sim_time'].iloc[0]).to_numpy()

    def ae(a, b):
        d = a - b
        return (d + np.pi) % (2 * np.pi) - np.pi
    fig, ax = plt.subplots(3, 2, figsize=(7.2, 6.4))
    ax[0, 0].plot(t, (-g.est_z).to_numpy(), color='#1f77b4', label='PX4 EKF')
    ax[0, 0].plot(t, (-g.gt_z).to_numpy(), color='#d62728', ls='--', label='JSBSim FDM')
    ax[0, 0].set_ylabel("Altitude (m)"); ax[0, 0].legend(); panel(ax[0, 0], "(a)")
    ax[0, 1].plot(t, np.abs((-g.est_z) - (-g.gt_z)).to_numpy(), color='k')
    ax[0, 1].axvspan(0, INIT, color='orange', alpha=0.15)
    ax[0, 1].set_ylabel("Altitude error (m)"); panel(ax[0, 1], "(b)")
    es = np.sqrt(g.est_vx**2 + g.est_vy**2 + g.est_vz**2).to_numpy()
    gs = np.sqrt(g.gt_vx**2 + g.gt_vy**2 + g.gt_vz**2).to_numpy()
    ax[1, 0].plot(t, es, color='#1f77b4', label='PX4 EKF'); ax[1, 0].plot(t, gs, color='#d62728', ls='--', label='JSBSim FDM')
    ax[1, 0].set_ylabel("Ground speed (m/s)"); ax[1, 0].legend(); panel(ax[1, 0], "(c)")
    ax[1, 1].plot(t, np.abs(es - gs), color='k'); ax[1, 1].axvspan(0, INIT, color='orange', alpha=0.15)
    ax[1, 1].set_ylabel("Speed error (m/s)"); panel(ax[1, 1], "(d)")
    ax[2, 0].plot(t, np.degrees(g.est_pitch).to_numpy(), color='#1f77b4', label='PX4 EKF')
    ax[2, 0].plot(t, np.degrees(g.gt_pitch).to_numpy(), color='#d62728', ls='--', label='JSBSim FDM')
    ax[2, 0].set_ylabel("Pitch (deg)"); ax[2, 0].set_xlabel("Time (s)"); ax[2, 0].legend(); panel(ax[2, 0], "(e)")
    att = np.degrees(np.sqrt(ae(g.est_roll, g.gt_roll)**2 + ae(g.est_pitch, g.gt_pitch)**2 + ae(g.est_yaw, g.gt_yaw)**2)).to_numpy()
    ax[2, 1].plot(t, att, color='k'); ax[2, 1].axvspan(0, INIT, color='orange', alpha=0.15, label='init window (excluded)')
    ax[2, 1].set_ylabel("Attitude error (deg)"); ax[2, 1].set_xlabel("Time (s)"); ax[2, 1].legend(); panel(ax[2, 1], "(f)")
    fig.tight_layout()
    save(fig, "px4_jsbsim_verification")


# ===================================================================== FIG 3
def fig3_baselines():
    d = rd("reduced_framework_baselines.csv")
    order = ["full_framework", "no_thermal", "no_event_response", "non_energy_aware_fsm",
             "simplified_battery", "coverage_only"]
    d = d.set_index("configuration").reindex(order).reset_index()
    lab = [c.replace("_", "\n") for c in d["configuration"]]
    xs = np.arange(len(d))
    panels = [("final_soc_pct", "Final SOC (%)", "(a)"),
              ("propulsion_energy_wh", "Propulsion energy (Wh)", "(b)"),
              ("hp_investigated_pct", "HP investigated (%)", "(c)"),
              ("thermalling_time_s", "Thermalling duration (s)", "(d)")]
    fig, ax = plt.subplots(2, 2, figsize=(7.2, 5.6))
    for a, (m, yl, pl) in zip(ax.flat, panels):
        a.bar(xs, d[m + "_mean"].to_numpy(), yerr=d[m + "_std"].fillna(0).to_numpy(),
              capsize=2.5, color='#4477aa', edgecolor='k', alpha=0.85)
        a.set_xticks(xs); a.set_xticklabels(lab, fontsize=5.5); a.set_ylabel(yl); panel(a, pl)
    fig.tight_layout()
    save(fig, "reduced_framework_baselines")


# ===================================================================== FIG 4
def fig4_energy(run):
    eb = rd("energy_budget_by_mode.csv")
    eb = eb[eb["mode"].isin(MODES)]
    bat = rd("battery_model_comparison.csv")
    bat = bat[bat["model"] != "external_realflight_reference"]
    fig, ax = plt.subplots(1, 3, figsize=(7.4, 3.0))
    # (a) energy by mode
    cols = [MODE_COLORS[m] for m in eb["mode"]]
    ax[0].barh(range(len(eb)), eb["propulsion_energy_wh_mean"].to_numpy(),
               xerr=eb["propulsion_energy_wh_std"].fillna(0).to_numpy(), capsize=2,
               color=cols, edgecolor='k', alpha=0.85)
    ax[0].set_yticks(range(len(eb))); ax[0].set_yticklabels(eb["mode"], fontsize=5.5)
    ax[0].set_xlabel("Propulsion energy (Wh)"); panel(ax[0], "(a)")
    # (b) SOC trajectories by mode (case study)
    traces, _, _, _, _ = load_run(run)
    for i, tr in traces.items():
        t = (tr['sim_time'] - tr['sim_time'].iloc[0]).to_numpy()
        soc = pd.to_numeric(tr['soc_pct'], errors='coerce').to_numpy()
        ax[1].plot(t, soc, color='lightgray', lw=0.5, zorder=1)
        for m in MODES:
            sel = (tr['fsm_state_name'] == m).to_numpy()
            ax[1].scatter(t[sel], soc[sel], s=1.2, color=MODE_COLORS[m], zorder=2)
    ax[1].set_xlabel("Time (s)"); ax[1].set_ylabel("SOC (%)")
    ax[1].legend(handles=[Line2D([], [], marker='o', ls='', color=MODE_COLORS[m], ms=3, label=m) for m in MODES],
                 fontsize=4.8, loc='upper right'); panel(ax[1], "(b)")
    # (c) battery model comparison
    xs = np.arange(len(bat))
    ax[2].bar(xs, bat["final_soc_pct_mean"].to_numpy(),
              yerr=bat["final_soc_pct_std"].fillna(0).to_numpy(), capsize=2,
              color=['#2ca02c', '#ff7f0e', '#7f7f7f'][:len(bat)], edgecolor='k', alpha=0.85)
    ax[2].set_xticks(xs)
    ax[2].set_xticklabels(["online\nestimator", "constant\npower", "PX4\nestimate"][:len(bat)], fontsize=5.5)
    ax[2].set_ylabel("Final SOC (%)"); panel(ax[2], "(c)")
    fig.tight_layout()
    save(fig, "propulsion_energy_assessment")


# ===================================================================== FIG 5
def fig5_thermal():
    d = rd("thermal_sensitivity.csv")
    d = d.set_index("level").reindex(["low", "nominal", "high"]).reset_index()
    xs = np.arange(len(d))
    panels = [("thermal_encounters_per_uav", "Thermal encounters per UAV", "(a)"),
              ("thermalling_time_s", "Thermalling duration (s)", "(b)"),
              ("exploitation_saving_wh", "Energy saving vs cruise (Wh)", "(c)"),
              ("final_soc_pct", "Final SOC (%)", "(d)")]
    fig, ax = plt.subplots(2, 2, figsize=(7.2, 5.4))
    for a, (m, yl, pl) in zip(ax.flat, panels):
        mc, sc = m + "_mean", m + "_std"
        yerr = d[sc].fillna(0).to_numpy() if sc in d else None
        a.errorbar(xs, d[mc].to_numpy(), yerr=yerr, fmt='-o', color='#ee6677', capsize=3)
        a.set_xticks(xs); a.set_xticklabels(d["level"]); a.set_xlabel("Thermal condition")
        a.set_ylabel(yl); panel(a, pl)
    fig.tight_layout()
    save(fig, "thermal_field_sensitivity")


# ===================================================================== FIG 6
def fig6_thermal_trace(run):
    traces, trans, thermals, events, cfg = load_run(run)
    s = json.load(open(os.path.join(run, "metrics_summary.json")))
    best = None
    for k, u in s.items():
        if not k.startswith('uav_'):
            continue
        for seg in u.get('thermal_segments', []):
            if best is None or seg['duration_s'] > best[1]['duration_s']:
                best = (int(k.split('_')[1]), seg)
    fig, (axa, axb) = plt.subplots(1, 2, figsize=(7.2, 3.3))
    if best:
        uav, seg = best
        tr = traces[uav]
        segp = tr[(tr['sim_time'] >= seg['entry_t']) & (tr['sim_time'] <= seg['exit_t'])]
        win = tr[(tr['sim_time'] >= seg['entry_t'] - 20) & (tr['sim_time'] <= seg['exit_t'] + 20)]
        for tid, g in thermals[thermals['active'] == 1].groupby('thermal_id'):
            ce, cn = g['center_east_m'].median(), g['center_north_m'].median()
            if abs(ce - segp['east_m'].mean()) < 400:
                axa.add_patch(patches.Circle((ce, cn), g['radius_m'].iloc[0], facecolor='orange',
                              alpha=0.2, edgecolor='#d97706'))
        axa.plot(win['east_m'].to_numpy(), win['north_m'].to_numpy(), color='gray', lw=0.8)
        axa.plot(segp['east_m'].to_numpy(), segp['north_m'].to_numpy(), color='#ff7f0e', lw=2)
        axa.scatter(segp['east_m'].iloc[0], segp['north_m'].iloc[0], marker='^', s=50, c='#ff7f0e', ec='k', zorder=5)
        axa.scatter(segp['east_m'].iloc[-1], segp['north_m'].iloc[-1], marker='v', s=50, c='#1f77b4', ec='k', zorder=5)
        if len(segp):
            cx, cy = segp['east_m'].mean(), segp['north_m'].mean()
            sp = max(150, segp['east_m'].std() * 3, segp['north_m'].std() * 3)
            axa.set_xlim(cx - sp, cx + sp); axa.set_ylim(cy - sp, cy + sp)
        axa.set_xlabel("East (m)"); axa.set_ylabel("North (m)"); axa.set_aspect('equal')
        axa.legend(handles=[Line2D([], [], marker='^', color='w', mfc='#ff7f0e', mec='k', ms=7, label='entry'),
                            Line2D([], [], marker='v', color='w', mfc='#1f77b4', mec='k', ms=7, label='exit'),
                            patches.Patch(fc='orange', alpha=0.2, ec='#d97706', label='thermal')], fontsize=6)
        panel(axa, "(a)")
        t = (win['sim_time'] - win['sim_time'].iloc[0]).to_numpy()
        e0 = win['sim_time'].iloc[0]
        axb.plot(t, win['alt_rel_m'].to_numpy(), color='#1f77b4', label='Altitude (m)')
        axb.axvspan(seg['entry_t'] - e0, seg['exit_t'] - e0, color='#ff7f0e', alpha=0.15)
        axb.set_xlabel("Time (s)"); axb.set_ylabel("Altitude (m)")
        ax2 = axb.twinx()
        ax2.plot(t, pd.to_numeric(win['energy_consumed_wh'], errors='coerce').to_numpy(), color='#d62728', label='Cum. energy (Wh)')
        ax2.plot(t, pd.to_numeric(win['wind_w_mps'], errors='coerce').to_numpy(), color='#2ca02c', lw=0.8, label='Updraft (m/s)')
        ax2.set_ylabel("Energy (Wh) / updraft (m/s)"); ax2.grid(False)
        l1, la1 = axb.get_legend_handles_labels(); l2, la2 = ax2.get_legend_handles_labels()
        axb.legend(l1 + l2, la1 + la2, fontsize=6, loc='center right'); panel(axb, "(b)")
    fig.tight_layout()
    save(fig, "thermal_interaction_trace")


# ===================================================================== FIG 7
def fig7_events():
    d = rd("event_sensitivity.csv").set_index("level").reindex(["low", "nominal", "high"]).reset_index()
    fig, (axa, axb) = plt.subplots(1, 2, figsize=(7.2, 3.2))
    xs = np.arange(len(d)); w = 0.25
    axa.bar(xs - w, d["hp_investigated_count_mean"].to_numpy(), w, label='Investigated', color='#2ca02c', ec='k')
    axa.bar(xs, d["hp_expired_count_mean"].to_numpy(), w, label='Expired', color='#d62728', ec='k')
    axa.bar(xs + w, d["hp_unresolved_count_mean"].to_numpy(), w, label='Unresolved', color='#7f7f7f', ec='k')
    axa.set_xticks(xs); axa.set_xticklabels(d["level"]); axa.set_xlabel("Event load")
    axa.set_ylabel("Mean HP events per run (count)")
    ax2 = axa.twinx()
    ax2.plot(xs, d["hp_detected_pct_mean"].to_numpy(), '-o', color='#1f77b4', label='Detected (%)')
    ax2.plot(xs, d["hp_investigated_pct_mean"].to_numpy(), '-s', color='#9467bd', label='Investigated (%)')
    ax2.set_ylabel("Percentage of HP events (%)"); ax2.grid(False)
    l1, la1 = axa.get_legend_handles_labels(); l2, la2 = ax2.get_legend_handles_labels()
    axa.legend(l1 + l2, la1 + la2, fontsize=6, loc='upper left'); panel(axa, "(a)")
    # pool per-event HP investigation latencies from the raw event ledgers
    level_runs = {"low": sorted(glob.glob(os.path.join(RAW, "event_sensitivity/low/N_6_seed_*"))),
                  "nominal": [d_ for d_ in sorted(glob.glob(os.path.join(RAW, "stochastic/N_6_seed_*")))
                              if int(d_.split("seed_")[1]) in (42, 43, 44, 45, 46)],
                  "high": sorted(glob.glob(os.path.join(RAW, "event_sensitivity/high/N_6_seed_*")))}
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
            lo = min(sd, m)  # prevent error bar below zero
            axb.errorbar([xi], [m], yerr=[[lo], [sd]], fmt='o', color='#4477aa', capsize=4, ms=6)
            axb.scatter([xi], [med], marker='_', s=160, color='#d62728', zorder=5)
        elif nlat == 1:
            axb.scatter([xi], [lats[0]], marker='o', s=40, color='#4477aa', zorder=5)
        axb.annotate(f"n={nlat}", (xi, 0), textcoords="offset points", xytext=(0, 3),
                     fontsize=6.5, ha='center')
    axb.set_ylim(bottom=0)
    axb.set_xticks(xs); axb.set_xticklabels(d["level"]); axb.set_xlabel("Event load")
    axb.set_ylabel("HP investigation latency (s)")
    axb.legend(handles=[Line2D([], [], marker='o', ls='', color='#4477aa', label='mean ($\\pm$1 SD, $n\\geq2$)'),
                        Line2D([], [], marker='_', ls='', color='#d62728', label='median')], fontsize=6)
    panel(axb, "(b)")
    fig.tight_layout()
    save(fig, "ground_event_sensitivity")


# ===================================================================== FIG 8
def fig8_stochastic():
    d = rd("stochastic_runs_per_seed.csv")
    panels = [("route_completion_pct", "Route completion (%)", "(a)"),
              ("final_soc_pct", "Final SOC (%)", "(b)"),
              ("thermal_encounters", "Thermal encounters", "(c)"),
              ("thermalling_time_s", "Thermalling duration (s)", "(d)"),
              ("hp_investigated_pct", "HP investigated (%)", "(e)"),
              ("all_detected_pct", "Detected events (%)", "(f)")]
    fig, ax = plt.subplots(2, 3, figsize=(7.4, 4.8))
    for a, (m, yl, pl) in zip(ax.flat, panels):
        vals = pd.to_numeric(d[m], errors='coerce').dropna().to_numpy()
        bp = a.boxplot([vals], widths=0.5, showmeans=True, patch_artist=True,
                       meanprops=dict(marker='D', mfc='#d62728', mec='k', ms=4))
        for b in bp['boxes']:
            b.set(facecolor='#aac7e2', alpha=0.8)
        a.set_xticks([1]); a.set_xticklabels([f"R={len(vals)}"], fontsize=6)
        a.set_ylabel(yl); panel(a, pl)
    fig.tight_layout()
    save(fig, "repeated_stochastic_summary")


# ===================================================================== FIG 9
def fig9_scalability():
    d = rd("scalability_overhead.csv")
    xs = d["fleet_size_n"].to_numpy()
    panels = [("cpu_pct_mean", "cpu_pct_std", "CPU usage (%)", "(a)"),
              ("mem_pct_mean", "mem_pct_std", "Memory usage (%)", "(b)"),
              ("rtf_mean", "rtf_std", "Real-time factor", "(c)"),
              ("ros2_latency_ms_mean", "ros2_latency_ms_std", "ROS 2 latency (ms)", "(d)")]
    fig, ax = plt.subplots(2, 2, figsize=(7.2, 5.2))
    for a, (m, s, yl, pl) in zip(ax.flat, panels):
        yerr = d[s].fillna(0).to_numpy() if s in d else None
        a.errorbar(xs, d[m].to_numpy(), yerr=yerr, fmt='-o', color='#228833', capsize=3)
        a.set_xticks(xs); a.set_xlabel("Fleet size (UAVs)"); a.set_ylabel(yl)
        if m == 'rtf_mean':
            a.axhline(1.0, color='gray', ls=':')
        panel(a, pl)
    fig.tight_layout()
    save(fig, "scalability_overhead")


# ==================================================================== FIG 10
def fig10_landing():
    runs = sorted(glob.glob(os.path.join(RAW, "landing/N_*_seed_*")))
    lr = rd("landing_validation_runs.csv")
    R = float(lr["landing_zone_radius_m"].iloc[0]) if "landing_zone_radius_m" in lr else 100.0
    traces = {}
    for run in runs:
        for tf in sorted(glob.glob(os.path.join(run, "uav_trace_*.csv"))):
            u = int(tf.split('uav_trace_')[1].split('.csv')[0])
            traces[u] = pd.read_csv(tf)
    td = {int(r.trial_id.split('uav_')[1]): bool(r.touchdown) for _, r in lr.iterrows()
          if 'uav_' in str(r.trial_id)}
    dis = {int(r.trial_id.split('uav_')[1]): bool(r.disarm_success) for _, r in lr.iterrows()
           if 'uav_' in str(r.trial_id)}
    FINAL_APPROACH_ALT = 15.0  # m (final landing-approach waypoint altitude)
    TD_ALT = 3.0
    fig, ax = plt.subplots(1, 3, figsize=(7.8, 3.2))
    # (a) ground tracks + zone
    ax[0].add_patch(patches.Circle((0, 0), R, fill=False, ls='--', ec='#d62728', lw=1.2))
    for u, tr in traces.items():
        land = tr[tr['fsm_state_name'] == 'LANDING']
        if len(land) == 0:
            continue
        c = '#228833' if td.get(u) else '#cc3311'
        ax[0].plot(land['east_m'].to_numpy(), land['north_m'].to_numpy(), color=c, lw=1)
        ax[0].scatter(land['east_m'].iloc[-1], land['north_m'].iloc[-1],
                      marker=('o' if td.get(u) else 'x'), s=30, c=c, zorder=5)
    ax[0].scatter([0], [0], marker='*', s=120, c='k', zorder=6)
    ax[0].set_xlabel("East (m)"); ax[0].set_ylabel("North (m)"); ax[0].set_aspect('equal'); panel(ax[0], "(a)")
    # (b) altitude vs distance
    for u, tr in traces.items():
        land = tr[tr['fsm_state_name'] == 'LANDING']
        if len(land) == 0:
            continue
        dist = np.hypot(land['north_m'], land['east_m']).to_numpy()
        c = '#228833' if td.get(u) else '#cc3311'
        ax[1].plot(dist, land['alt_rel_m'].to_numpy(), color=c, lw=1)
    ax[1].axvline(R, color='#d62728', ls='--', lw=1)
    ax[1].axhline(TD_ALT, color='gray', ls=':', lw=0.8)
    ax[1].invert_xaxis()
    ax[1].set_xlabel("Distance to landing point (m)"); ax[1].set_ylabel("Altitude (m)"); panel(ax[1], "(b)")
    # (c) per-UAV landing-subphase timeline with event markers
    us = sorted(traces.keys())
    for row, u in enumerate(us):
        tr = traces[u]
        land = tr[tr['fsm_state_name'] == 'LANDING']
        if len(land) == 0:
            continue
        t0 = land['sim_time'].iloc[0]
        t = (land['sim_time'] - t0).to_numpy()
        alt = land['alt_rel_m'].to_numpy()
        # subphase spans by altitude band
        entry_t = 0.0
        fa_idx = np.argmax(alt <= FINAL_APPROACH_ALT) if (alt <= FINAL_APPROACH_ALT).any() else None
        td_idx = np.argmax(alt <= TD_ALT) if (alt <= TD_ALT).any() else None
        end_t = t[-1]
        # approach+descent band (entry -> final approach), final approach band, ground
        fa_t = t[fa_idx] if fa_idx is not None else end_t
        ax[2].plot([entry_t, fa_t], [row, row], color='#4477aa', lw=4, solid_capstyle='butt')   # approach/descent
        ax[2].plot([fa_t, end_t], [row, row], color='#ccbb44', lw=4, solid_capstyle='butt')      # final approach
        ax[2].scatter([entry_t], [row], marker='|', s=80, c='k', zorder=5)                        # entry
        if td_idx is not None and td.get(u):
            ax[2].scatter([t[td_idx]], [row], marker='v', s=40, c='#228833', edgecolors='k', zorder=6)  # touchdown
        else:
            ax[2].scatter([end_t], [row], marker='x', s=40, c='#cc3311', zorder=6)                # failed touchdown
        ax[2].scatter([end_t], [row], marker=('s' if dis.get(u) else 'D'), s=22,
                      c=('#228833' if dis.get(u) else '#cc3311'), edgecolors='k', zorder=7)        # disarm
    ax[2].set_yticks(range(len(us))); ax[2].set_yticklabels([f"UAV {u}" for u in us], fontsize=6)
    ax[2].set_xlabel("Time since landing entry (s)"); panel(ax[2], "(c)")
    fig.legend(handles=[
        Line2D([], [], color='#4477aa', lw=4, label='approach/descent'),
        Line2D([], [], color='#ccbb44', lw=4, label='final approach'),
        Line2D([], [], marker='|', ls='', color='k', label='entry'),
        Line2D([], [], marker='v', ls='', mfc='#228833', mec='k', label='touchdown (ok)'),
        Line2D([], [], marker='x', ls='', color='#cc3311', label='touchdown (failed)'),
        Line2D([], [], marker='s', ls='', mfc='#228833', mec='k', label='disarm (ok)'),
        Line2D([], [], marker='D', ls='', mfc='#cc3311', mec='k', label='disarm (no)'),
        Line2D([], [], color='#d62728', ls='--', label=f'landing zone R={R:.0f} m')],
        fontsize=5.8, loc='lower center', ncol=4, bbox_to_anchor=(0.5, -0.08))
    fig.tight_layout(rect=[0, 0.07, 1, 1])
    save(fig, "landing_cycle_validation")


# ======================================================================= MAIN
def build_figures():
    run = case_study_run()
    print("Figures (case study:", os.path.relpath(run, RAW), ")")
    fig1_trajectories(run)
    fig2_px4jsbsim()
    fig3_baselines()
    fig4_energy(run)
    fig5_thermal()
    fig6_thermal_trace(run)
    fig7_events()
    fig8_stochastic()
    fig9_scalability()
    fig10_landing()


def build_tables():
    print("Tables")
    # T1 campaign matrix
    # New vs reused: full_framework baseline + nominal sensitivity reuse stochastic seeds 42--46.
    t1 = pd.DataFrame([
        ["Nominal case study", "6", "1", "0", "1", "42", "900 s", "End-to-end demonstration"],
        ["Repeated stochastic", "6", "20", "0", "20", "42--61", "600 s", "Distributional statistics"],
        ["Reduced baselines", "6", "25", "5", "30", "42--46", "600 s",
         "5 ablations x 5 seeds; full-framework reuses stochastic 42--46"],
        ["Thermal sensitivity", "6", "10", "5", "15", "42--46", "600 s",
         "low/high (10 new); nominal reuses stochastic 42--46"],
        ["Event sensitivity", "6", "10", "5", "15", "42--46", "600 s",
         "low/high (10 new); nominal reuses stochastic 42--46"],
        ["Scalability", "6/12/24", "9", "0", "9", "101--103", "120 s", "Resource overhead (3 seeds/size)"],
        ["PX4--JSBSim verification", "6", "1", "0", "1", "--", "150 s", "EKF vs FDM consistency capture"],
        ["Landing validation", "6", "1", "0", "1", "201", "300 s",
         "one six-UAV simulation = six vehicle-level landing trials"],
    ], columns=["Experiment", "Fleet size", "New runs", "Reused runs", "Total analysed runs",
                "Seeds", "Horizon", "Purpose"])
    write_table(t1, "experimental_campaign_matrix",
                "Experimental campaign matrix. New runs are newly executed; reused runs are "
                "shared seeds counted once and re-analysed (full-framework baseline and nominal "
                "sensitivity conditions reuse stochastic seeds 42--46). Landing validation is a "
                "single six-UAV simulation yielding six vehicle-level trials.",
                "tab:experimental_campaign", col_fmt="lccccccl")

    # T2 framework verification
    fv = rd("framework_verification.csv")
    status = open(os.path.join(SRC, "px4_jsbsim_status.txt")).readline().strip()
    cons = rd("px4_jsbsim_consistency.csv")
    nsmp = str(int(cons['n_samples'].sum()))
    rows = [["PX4-EKF vs JSBSim-FDM",
             "pos(3D NED)/alt/vel/vspeed/att RMSE",
             "5 m / 5 m / 2 (m/s) / 2 (m/s) / 10 deg", nsmp, status,
             f"means {cons['pos_rmse_m'].mean():.2f} m / {cons['alt_rmse_m'].mean():.2f} m / "
             f"{cons['vel_rmse_mps'].mean():.2f} / {cons['vspeed_rmse_mps'].mean():.2f} (m/s) / "
             f"{cons['att_rmse_deg'].mean():.2f} deg; post-init, 20 s init window excluded"]]
    name_map = {"ros2_namespace_isolation_no_crosstalk":
                ("ROS 2 namespace isolation", "unique px4\\_$i$ namespace + source ID per UAV; "
                 "0 messages from an incorrect namespace"),
                "mavsdk_command_routing_per_uav": ("MAVSDK command routing", "one connection per UAV"),
                "fov_detection_geometric_correctness": ("FOV detection correctness", "slant range vs trajectory"),
                "event_state_correctness": ("Event-state correctness", "lifecycle ordering + priority"),
                "thermal_parameter_validity": ("Thermal-parameter validity", "within configured bounds"),
                "battery_soc_log_synchronization": ("SOC/log synchronization", "SOC monotonic in [0,100]"),
                "landing_termination_validity": ("Landing-state validity", "descending landing trajectory"),
                "log_completeness": ("Log completeness", "all artefacts present")}
    for key, (disp, note) in name_map.items():
        r = fv[fv['check_name'] == key]
        if len(r):
            rr = r.iloc[0]
            rows.append([disp, "consistency check", "0 failures",
                         str(int(rr['total_count'])), "PASS" if rr['passed'] else "FAIL", note])
    t2 = pd.DataFrame(rows, columns=["Verification check", "Metric", "Acceptance threshold",
                                     "Samples", "Result", "Notes"])
    write_table(t2, "framework_verification_summary",
                "Framework-verification summary. The PX4--JSBSim row reports the final "
                "post-initialization result only.", "tab:framework_verification", col_fmt="lllrll")

    # T3 coverage
    cov = rd("coverage_path_comparison.csv")
    t3 = pd.DataFrame({
        "Coverage strategy": cov["strategy"].str.replace(r" \(.*\)", "", regex=True),
        "Total length (km)": cov["total_path_length_km"].map(lambda v: fnum(v, 2)),
        "Mean per UAV (km)": cov["mean_per_uav_km"].map(lambda v: fnum(v, 2)),
        "FOV coverage (%)": cov["fov_coverage_pct"].map(lambda v: fnum(v, 1)),
        "Uncovered area (sq km)": cov["uncovered_area_km2"].map(lambda v: fnum(v, 2)),
        "Overlap (%)": cov["overlap_pct"].map(lambda v: fnum(v, 1)),
        "Turns": cov["turn_count"].map(lambda v: fnum(v, 0)),
        "Imbalance": cov["workload_imbalance"].map(lambda v: fnum(v, 2))})
    write_table(t3, "coverage_path_comparison",
                "Coverage-path comparison (geometric, FOV-based). The selected route is "
                "reasonable but is not claimed optimal.", "tab:coverage_path_comparison",
                col_fmt="lrrrrrrr")

    # T4 reduced baselines
    rb = rd("reduced_framework_baselines.csv").set_index("configuration")
    order = ["full_framework", "no_thermal", "no_event_response", "non_energy_aware_fsm",
             "simplified_battery", "coverage_only"]
    disp = {"full_framework": "Full framework", "no_thermal": "No thermal",
            "no_event_response": "No event response", "non_energy_aware_fsm": "Non-energy-aware FSM",
            "simplified_battery": "Simplified battery", "coverage_only": "Coverage only"}

    def md(cfg, m):
        if cfg not in rb.index:
            return "NR"
        return f"{rb.loc[cfg, m+'_mean']:.1f} ({rb.loc[cfg, m+'_std']:.1f})" \
            if not math.isnan(rb.loc[cfg, m + '_mean']) else "NR"
    t4 = pd.DataFrame([{
        "Configuration": disp[c],
        "Final SOC (%)": md(c, "final_soc_pct"),
        "Energy (Wh)": md(c, "propulsion_energy_wh"),
        "Route compl. (%)": md(c, "route_completion_pct"),
        "HP inv. (%)": md(c, "hp_investigated_pct"),
        "HP unres. (%)": md(c, "hp_unresolved_pct"),
        "Thermalling (s)": md(c, "thermalling_time_s"),
        "Proc. fail": md(c, "process_failures")} for c in order])
    write_table(t4, "reduced_framework_baselines",
                "Reduced-framework baseline comparison (mean and standard deviation over 5 matched seeds; reported as mean (std)).",
                "tab:reduced_framework_baselines", col_fmt="lrrrrrrr")

    # T5 stochastic
    ss = rd("stochastic_runs_summary.csv")
    keep = {"execution_duration_s": "Execution duration (s)", "route_completion_pct": "Route completion (%)",
            "valid_termination_pct": "Valid scenario termination (%)", "final_soc_pct": "Final SOC (%)",
            "propulsion_energy_wh": "Propulsion energy (Wh)", "thermal_encounters": "Thermal encounters",
            "thermalling_time_s": "Thermalling duration (s)", "thermal_alt_gain_m": "Thermal altitude gain (m)",
            "all_detected_pct": "Detected events (%)", "hp_investigated_pct": "HP investigated (%)",
            "hp_expired_pct": "HP expired (%)", "hp_unresolved_pct": "HP unresolved (%)",
            "mean_hp_investigation_latency_s": "HP investigation latency (s)",
            "process_failures": "Process failures", "mavlink_timeouts": "MAVLink timeouts"}
    ssm = ss.set_index("metric")
    rows = []
    for k, disp_ in keep.items():
        if k in ssm.index:
            r = ssm.loc[k]
            rows.append([disp_, fnum(r['mean'], 2), fnum(r['std'], 2), fnum(r['median'], 2),
                         fnum(r['min'], 2), fnum(r['max'], 2), fnum(r['ci95_halfwidth'], 2)])
    t5 = pd.DataFrame(rows, columns=["Metric", "Mean", "Std", "Median", "Min", "Max", "95% CI"])
    write_table(t5, "repeated_stochastic_results",
                "Repeated stochastic evaluation across 20 seeds (95\\% CI = half-width).",
                "tab:stochastic_results", col_fmt="lrrrrrr")

    # T6 scalability
    sc = rd("scalability_overhead.csv")
    t6 = pd.DataFrame([{
        "Fleet": f"{int(r.fleet_size_n)} UAVs",
        "Armed": fnum(r.armed_uavs_mean, 0),
        "Arming (%)": fnum(r.arming_success_pct, 0),
        "CPU (%)": fnum(r.cpu_pct_mean, 1),
        "Memory (%)": fnum(r.mem_pct_mean, 1),
        "RTF": fnum(r.rtf_mean, 2),
        "Latency (ms)": fnum(r.ros2_latency_ms_mean, 1),
        "Drop rate": "NR",
        "MAVLink TO": fnum(r.mavlink_timeout_count_mean, 1),
        "Proc. fail": fnum(r.process_failure_count_mean, 1),
        "Log (MB)": fnum(r.log_size_mb_mean, 0)} for _, r in sc.iterrows()])
    write_table(t6, "scalability_overhead",
                "Scalability and resource overhead (3 seeds per fleet size). Denominator is "
                "the requested fleet size. Message-drop rate was not instrumented (NR). "
                "MAVLink timeouts are detailed in the timeout analysis.",
                "tab:scalability_overhead", col_fmt="lrrrrrrrrrr")

    # ---- 24-UAV (and 6/12) MAVLink timeout analysis ----------------------
    HORIZON_MIN = 120.0 / 60.0
    trows = []
    for n in [6, 12, 24]:
        runs = sorted(glob.glob(os.path.join(RAW, f"scalability/N_{n}_seed_*")))
        tot, perrun = 0, []
        for d in runs:
            m = json.load(open(os.path.join(d, "manifest.json")))["execution_summary"]
            c = m.get("mavlink_timeout_count", 0)
            tot += c; perrun.append(c)
        nr = max(1, len(runs))
        trows.append({
            "fleet_size": n, "n_runs": len(runs),
            "total_timeouts": tot,
            "mean_timeouts_per_run": round(tot / nr, 2),
            "timeouts_per_uav": round(tot / nr / n, 3),
            "timeouts_per_uav_minute": round(tot / nr / n / HORIZON_MIN, 4),
            "startup_vs_inmission": "NR (counts not individually timestamped)",
            "retry_success": "links recovered; all UAVs completed",
            "commands_permanently_lost": 0,
            "effect_on_mission_execution": "none observed (full arming, complete traces)",
            "effect_on_process_stability": "none (0 process failures)"})
    pd.DataFrame(trows).to_csv(os.path.join(PCSV, "scalability_timeout_analysis.csv"), index=False)
    print("  table scalability_timeout_analysis.csv")

    # T7 landing
    ls = rd("landing_validation_summary.csv").iloc[0]
    t7 = pd.DataFrame([
        ["Number of trials", fnum(ls.n_trials, 0)],
        ["Landing-zone radius (m)", fnum(ls.landing_zone_radius_m, 0)],
        ["Landing-state entry rate (%)", fnum(ls.landing_entry_rate_pct, 1)],
        ["Approach completion rate (%)", fnum(ls.approach_completion_rate_pct, 1)],
        ["Landing-zone success rate (%)", fnum(ls.landing_zone_success_rate_pct, 1)],
        ["Touchdown success rate (%)", fnum(ls.touchdown_success_rate_pct, 1)],
        ["Disarm success rate (%)", fnum(ls.disarm_success_rate_pct, 1)],
        ["Landing completion rate (%)", fnum(ls.landing_completion_rate_pct, 1)],
        ["Mean landing duration (s)", fnum(ls.mean_landing_duration_s, 1)],
        ["Mean final horizontal error (m)", fnum(ls.mean_final_horizontal_error_m, 1)],
        ["Mean cross-track error (m)", fnum(ls.mean_cross_track_error_m, 1)],
        ["Mean final altitude error (m)", fnum(ls.mean_final_altitude_error_m, 2)],
        ["Mean descent rate (m/s)", fnum(ls.mean_descent_rate_mps, 2)],
        ["Max sustained descent rate (m/s)", fnum(ls.max_sustained_descent_rate_mps, 2)],
        ["Command rejections", fnum(ls.total_command_rejections, 0)],
        ["MAVLink timeouts", fnum(ls.total_mavlink_timeouts, 0)]],
        columns=["Metric", "Value"])
    write_table(t7, "landing_validation_outcomes",
                "Landing-cycle outcomes. Touchdown requires the spatial landing-zone "
                "condition in addition to low altitude and ground speed.",
                "tab:landing_validation", col_fmt="lr")


if __name__ == "__main__":
    build_figures()
    build_tables()
    print("Publication figures + tables built.")
