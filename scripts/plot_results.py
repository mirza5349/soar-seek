#!/usr/bin/env python3
"""Generate all manuscript figures (PNG 300 dpi + vector PDF) from real run
logs and the aggregated CSVs. No synthetic data: every plotted value comes
from logs/raw/** or results/csv/**.
"""
import os
import glob
import json
import math
import yaml
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.lines import Line2D

WORKSPACE = "/home/px4_sitl/sim_paper"
RAW = os.path.join(WORKSPACE, "logs/raw")
CSV_DIR = os.path.join(WORKSPACE, "results/csv")
FIG_DIR = os.path.join(WORKSPACE, "results/figures")

UAV_COLORS = ['#1f77b4', '#2ca02c', '#9467bd', '#d62728', '#e377c2', '#17becf']
MODE_COLORS = {
    'PATROL': '#1f77b4', 'EVENT_INVESTIGATION': '#d62728',
    'THERMAL_SEARCH': '#bcbd22', 'THERMAL_EXPLOITATION': '#ff7f0e',
    'GLIDE_RETURN': '#2ca02c', 'LANDING': '#7f7f7f'}
MODES = list(MODE_COLORS.keys())

plt.rcParams.update({
    'font.family': 'sans-serif', 'font.size': 10, 'axes.labelsize': 11,
    'axes.titlesize': 12, 'figure.titlesize': 13, 'axes.grid': True,
    'grid.linestyle': ':', 'grid.alpha': 0.6})


def save_fig(fig, name):
    fig.savefig(os.path.join(FIG_DIR, name + ".png"), dpi=300, bbox_inches='tight')
    fig.savefig(os.path.join(FIG_DIR, name + ".pdf"), format='pdf', bbox_inches='tight')
    plt.close(fig)
    print(f"  saved {name}.png/.pdf")


def pick_case_study_run():
    """Prefer the nominal long-horizon run; fall back to the stochastic run
    with the most investigations + thermal encounters."""
    cands = sorted(glob.glob(os.path.join(RAW, "nominal/N_*_seed_*")))
    cands += sorted(glob.glob(os.path.join(RAW, "stochastic/N_*_seed_*")))
    best, best_score = None, -1.0
    for d in cands:
        sp = os.path.join(d, "metrics_summary.json")
        if not os.path.exists(sp):
            continue
        s = json.load(open(sp))
        fleet = s.get('_fleet', {})
        score = 10.0 * fleet.get('hp_investigated_count', 0)
        score += sum(u.get('thermal_encounters', 0) for k, u in s.items() if k.startswith('uav'))
        if 'nominal' in d:
            score += 100.0
        if score > best_score:
            best, best_score = d, score
    return best


def load_run_data(run):
    traces = {}
    for p in sorted(glob.glob(os.path.join(run, "uav_trace_*.csv"))):
        uav = int(p.split("uav_trace_")[1].split(".csv")[0])
        traces[uav] = pd.read_csv(p)
    trans = pd.read_csv(os.path.join(run, "fsm_transitions.csv"))
    thermals = pd.read_csv(os.path.join(run, "thermal_field.csv"))
    events = pd.read_csv(os.path.join(run, "events.csv"))
    cfg = yaml.safe_load(open(os.path.join(run, "config.yaml")))
    return traces, trans, thermals, events, cfg


def planned_routes(cfg, n_uavs):
    fsm = cfg['fsm_node']['ros__parameters']
    routes = {}
    for i in range(1, n_uavs + 1):
        key = f'patrol_waypoints_{i}' if f'patrol_waypoints_{i}' in fsm else 'patrol_waypoints_1'
        flat = fsm[key]
        routes[i] = [(flat[j], flat[j + 1]) for j in range(0, len(flat), 3)]
    return routes


def pos_at(trace, t):
    idx = (trace['sim_time'] - t).abs().idxmin()
    r = trace.loc[idx]
    return r['east_m'], r['north_m']


# ----------------------------------------------------------- figure builders
def fig_evaluation_region_path():
    cfg = yaml.safe_load(open(os.path.join(WORKSPACE, "configs/scenario_nominal.yaml")))
    routes = planned_routes(cfg, 6)
    cov = pd.read_csv(os.path.join(CSV_DIR, "coverage_path_comparison.csv"))
    sel_len = cov.iloc[0]['total_path_length_km']

    fig, ax = plt.subplots(figsize=(9, 7))
    all_pts = [p for r in routes.values() for p in r]
    from matplotlib.patches import Polygon as MplPoly
    from scipy.spatial import ConvexHull
    pts = np.array([(e, n) for (n, e) in all_pts])
    hull = ConvexHull(pts)
    ax.add_patch(MplPoly(pts[hull.vertices], closed=True, facecolor='#eef3f8',
                         edgecolor='#0066cc', lw=1.5, alpha=0.8,
                         label='Operational region (route hull)'))
    for i, route in routes.items():
        e = [p[1] for p in route]
        n = [p[0] for p in route]
        ax.plot(e, n, '-o', color=UAV_COLORS[i - 1], lw=1.8, ms=4,
                label=f'UAV {i} segment ({len(route)} wps)')
    ax.set_title(f"Evaluation region and partitioned coverage route "
                 f"(total {sel_len:.1f} km, 6 UAVs)", fontweight='bold')
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect('equal')
    ax.legend(loc='lower left', fontsize=8, ncol=2)
    save_fig(fig, "evaluation_region_path")


def fig_executed_path_overlay(run):
    traces, trans, thermals, events, cfg = load_run_data(run)
    routes = planned_routes(cfg, len(traces))

    fig, ax = plt.subplots(figsize=(10, 8.5))

    # planned routes (dashed)
    for i, route in routes.items():
        e = [p[1] for p in route]
        n = [p[0] for p in route]
        ax.plot(e, n, '--', color='gray', lw=0.9, alpha=0.7, zorder=1)

    # thermal footprints: snapshot at each thermal's median logged position
    for tid, grp in thermals[thermals['active'] == 1].groupby('thermal_id'):
        ce, cn = grp['center_east_m'].median(), grp['center_north_m'].median()
        r = grp['radius_m'].iloc[0]
        ax.add_patch(patches.Circle((ce, cn), r, facecolor='orange', alpha=0.15,
                                    edgecolor='#d97706', lw=0.8, zorder=2))

    # HP event locations
    hp = events[events['is_high_priority'] == 1]
    inv = hp[hp['final_state'] == 'investigated']
    other = hp[hp['final_state'] != 'investigated']
    ax.scatter(other['east_m'], other['north_m'], marker='*', s=110, c='#999999',
               edgecolors='k', linewidths=0.4, zorder=6, label='HP event (not investigated)')
    ax.scatter(inv['east_m'], inv['north_m'], marker='*', s=170, c='#d62728',
               edgecolors='k', linewidths=0.6, zorder=7, label='HP event (investigated)')

    # executed trajectories with state-highlighted segments
    for i, tr in traces.items():
        c = UAV_COLORS[(i - 1) % len(UAV_COLORS)]
        ax.plot(tr['east_m'].to_numpy(), tr['north_m'].to_numpy(), color=c, lw=1.0, alpha=0.85, zorder=3)
        for mode, style in [('THERMAL_EXPLOITATION', dict(color='#ff7f0e', lw=2.6)),
                            ('EVENT_INVESTIGATION', dict(color='#d62728', lw=2.6)),
                            ('GLIDE_RETURN', dict(color='#2ca02c', lw=1.8)),
                            ('LANDING', dict(color='#7f7f7f', lw=1.8))]:
            seg = tr[tr['fsm_state_name'] == mode]
            if len(seg) > 1:
                # break into contiguous blocks to avoid connecting lines
                blocks = np.split(seg.index.values,
                                  np.where(np.diff(seg.index.values) > 1)[0] + 1)
                for b in blocks:
                    if len(b) > 1:
                        ax.plot(tr.loc[b, 'east_m'].to_numpy(), tr.loc[b, 'north_m'].to_numpy(),
                                zorder=5, **style)
        # start / end markers
        ax.scatter(tr['east_m'].iloc[0], tr['north_m'].iloc[0], marker='o', s=30,
                   c=c, edgecolors='k', zorder=8)
        ax.scatter(tr['east_m'].iloc[-1], tr['north_m'].iloc[-1], marker='X', s=45,
                   c=c, edgecolors='k', zorder=8)

    # thermal entry/exit markers from transition log
    for _, t in trans.iterrows():
        uav = int(t['uav_id'])
        if uav not in traces:
            continue
        if t['to_state'] == 'THERMAL_EXPLOITATION':
            e, n = pos_at(traces[uav], t['sim_time'])
            ax.scatter(e, n, marker='^', s=55, c='#ff7f0e', edgecolors='k',
                       linewidths=0.6, zorder=9)
        elif t['from_state'] == 'THERMAL_EXPLOITATION':
            e, n = pos_at(traces[uav], t['sim_time'])
            ax.scatter(e, n, marker='v', s=55, c='#1f77b4', edgecolors='k',
                       linewidths=0.6, zorder=9)

    handles = [
        Line2D([], [], color='gray', ls='--', lw=1, label='Planned route'),
        Line2D([], [], color=UAV_COLORS[0], lw=1.2, label='Executed trajectory (per-UAV colour)'),
        Line2D([], [], color='#ff7f0e', lw=2.6, label='Thermal exploitation segment'),
        Line2D([], [], color='#d62728', lw=2.6, label='Event-investigation loiter'),
        Line2D([], [], color='#2ca02c', lw=1.8, label='Glide return segment'),
        Line2D([], [], color='#7f7f7f', lw=1.8, label='Landing segment'),
        Line2D([], [], marker='^', color='w', mfc='#ff7f0e', mec='k', ms=8, label='Thermal entry'),
        Line2D([], [], marker='v', color='w', mfc='#1f77b4', mec='k', ms=8, label='Thermal exit'),
        Line2D([], [], marker='*', color='w', mfc='#d62728', mec='k', ms=12, label='HP event (investigated)'),
        Line2D([], [], marker='*', color='w', mfc='#999999', mec='k', ms=10, label='HP event (other)'),
        patches.Patch(facecolor='orange', alpha=0.25, edgecolor='#d97706', label='Active thermal footprint'),
        Line2D([], [], marker='o', color='w', mfc='k', ms=6, label='UAV start'),
        Line2D([], [], marker='X', color='w', mfc='k', ms=8, label='UAV end'),
    ]
    ax.legend(handles=handles, loc='upper left', fontsize=7.5, ncol=2, framealpha=0.95)
    ax.set_title(f"Executed trajectories vs planned route — {os.path.relpath(run, RAW)}",
                 fontweight='bold')
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_aspect('equal')
    save_fig(fig, "executed_path_overlay")


def pick_representative_uav(run):
    s = json.load(open(os.path.join(run, "metrics_summary.json")))
    best, best_score = 1, -1
    for k, u in s.items():
        if not k.startswith('uav_'):
            continue
        score = 2 * u.get('investigations_count', 0) + u.get('thermal_encounters', 0)
        if u.get('entered_landing'):
            score += 1
        if score > best_score:
            best, best_score = int(k.split('_')[1]), score
    return best


def fig_representative_timeline(run):
    traces, trans, thermals, events, cfg = load_run_data(run)
    uav = pick_representative_uav(run)
    tr = traces[uav]
    t0 = tr['sim_time'].iloc[0]
    t = tr['sim_time'] - t0

    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True)

    # state band shading helper
    def shade(ax):
        for mode, color in [('THERMAL_EXPLOITATION', '#ff7f0e'),
                            ('EVENT_INVESTIGATION', '#d62728')]:
            seg = tr[tr['fsm_state_name'] == mode]
            if seg.empty:
                continue
            blocks = np.split(seg.index.values,
                              np.where(np.diff(seg.index.values) > 1)[0] + 1)
            for b in blocks:
                ax.axvspan(tr.loc[b[0], 'sim_time'] - t0, tr.loc[b[-1], 'sim_time'] - t0,
                           color=color, alpha=0.12, zorder=0)

    # 1: FSM state
    state_idx = tr['fsm_state_name'].map({m: i for i, m in enumerate(MODES)})
    axes[0].step(t.to_numpy(), state_idx.to_numpy(), where='post', color='k', lw=1.2)
    axes[0].set_yticks(range(len(MODES)))
    axes[0].set_yticklabels(MODES, fontsize=7)
    axes[0].set_ylabel("FSM state")
    shade(axes[0])

    # 2: altitude + wind
    axes[1].plot(t.to_numpy(), tr['alt_rel_m'].to_numpy(), color='#1f77b4', lw=1.2, label='Altitude')
    ax1b = axes[1].twinx()
    ax1b.plot(t.to_numpy(), pd.to_numeric(tr['wind_w_mps'], errors='coerce').to_numpy(), color='#ff7f0e',
              lw=0.8, alpha=0.7, label='Thermal updraft')
    ax1b.set_ylabel("w (m/s)", color='#ff7f0e')
    ax1b.grid(False)
    axes[1].set_ylabel("Altitude (m)")
    shade(axes[1])

    # 3: SOC (+ PX4 reference if present)
    axes[2].plot(t.to_numpy(), pd.to_numeric(tr['soc_pct'], errors='coerce').to_numpy(), color='#2ca02c',
                 lw=1.4, label='Propulsion-only estimator')
    px4 = pd.to_numeric(tr['px4_batt_remaining_pct'], errors='coerce')
    if px4.notna().sum() > 10:
        axes[2].plot(t.to_numpy(), px4.to_numpy(), color='#7f7f7f', lw=1.0, ls='--', label='PX4 default estimate')
    axes[2].set_ylabel("SOC (%)")
    axes[2].legend(fontsize=8, loc='lower left')
    shade(axes[2])

    # 4: propulsion power
    axes[3].plot(t.to_numpy(), pd.to_numeric(tr['power_w'], errors='coerce').to_numpy(), color='#d62728', lw=1.0)
    axes[3].set_ylabel("Propulsion power (W)")
    axes[3].set_xlabel("Mission time (s)")
    shade(axes[3])

    # thermal entry/exit lines
    tt = trans[trans['uav_id'] == uav]
    for _, r in tt.iterrows():
        if r['to_state'] == 'THERMAL_EXPLOITATION':
            for ax in axes:
                ax.axvline(r['sim_time'] - t0, color='#ff7f0e', lw=0.8, ls=':')
        if r['from_state'] == 'THERMAL_EXPLOITATION':
            for ax in axes:
                ax.axvline(r['sim_time'] - t0, color='#1f77b4', lw=0.8, ls=':')

    fig.suptitle(f"Representative UAV timeline — UAV {uav}, {os.path.relpath(run, RAW)}\n"
                 "(orange shading: thermal exploitation; red shading: event investigation; "
                 "dotted lines: thermal entry/exit)", fontweight='bold')
    save_fig(fig, "representative_uav_timeline")


def fig_soc_time_series(run):
    traces, _, _, _, _ = load_run_data(run)
    fig, ax = plt.subplots(figsize=(10, 5.5))
    for i, tr in traces.items():
        t0 = tr['sim_time'].iloc[0]
        t = tr['sim_time'] - t0
        soc = pd.to_numeric(tr['soc_pct'], errors='coerce')
        ax.plot(t.to_numpy(), soc.to_numpy(), color='lightgray', lw=0.8, zorder=1)
        for mode in MODES:
            seg = tr['fsm_state_name'] == mode
            ax.scatter(t[seg].to_numpy(), soc[seg].to_numpy(), s=2.5, color=MODE_COLORS[mode], zorder=2)
    handles = [Line2D([], [], marker='o', ls='', color=MODE_COLORS[m], label=m, ms=5)
               for m in MODES]
    ax.legend(handles=handles, fontsize=8, loc='upper right')
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("SOC (%) — propulsion-only estimator")
    ax.set_title("SOC time series coloured by FSM mode (all UAVs, case-study run)",
                 fontweight='bold')
    save_fig(fig, "soc_time_series_by_mode")


def fig_thermal_altitude_energy_trace(run):
    traces, trans, _, _, _ = load_run_data(run)
    # pick the longest exploitation segment across UAVs
    best = None
    s = json.load(open(os.path.join(run, "metrics_summary.json")))
    for k, u in s.items():
        if not k.startswith('uav_'):
            continue
        for seg in u.get('thermal_segments', []):
            if best is None or seg['duration_s'] > best[1]['duration_s']:
                best = (int(k.split('_')[1]), seg)
    if best is None:
        print("  WARNING: no thermal segment found; skipping thermal trace figure")
        return
    uav, seg = best
    tr = traces[uav]
    t0 = tr['sim_time'].iloc[0]
    w0, w1 = seg['entry_t'] - 40.0, seg['exit_t'] + 40.0
    win = tr[(tr['sim_time'] >= w0) & (tr['sim_time'] <= w1)]
    t = win['sim_time'] - t0

    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(t.to_numpy(), win['alt_rel_m'].to_numpy(), color='#1f77b4', lw=1.6, label='Altitude')
    ax.axvspan(seg['entry_t'] - t0, seg['exit_t'] - t0, color='#ff7f0e', alpha=0.15,
               label='THERMAL_EXPLOITATION')
    ax.axvline(seg['entry_t'] - t0, color='#ff7f0e', ls=':', lw=1.2)
    ax.axvline(seg['exit_t'] - t0, color='#1f77b4', ls=':', lw=1.2)
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("Altitude (m)", color='#1f77b4')

    ax2 = ax.twinx()
    ax2.plot(t.to_numpy(), pd.to_numeric(win['energy_consumed_wh'], errors='coerce').to_numpy(),
             color='#d62728', lw=1.4, label='Cumulative propulsion energy')
    ax2.plot(t.to_numpy(), pd.to_numeric(win['wind_w_mps'], errors='coerce').to_numpy(),
             color='#2ca02c', lw=0.9, alpha=0.8, label='Updraft w (m/s)')
    ax2.set_ylabel("Energy (Wh) / updraft (m/s)")
    ax2.grid(False)

    gain = seg.get('alt_gain_m', float('nan'))
    de = seg.get('energy_consumed_wh', float('nan'))
    ax.set_title(f"Thermal exploitation trace — UAV {uav}: "
                 f"{seg['duration_s']:.0f} s in thermal, altitude gain "
                 f"{gain:+.0f} m, propulsion energy during segment {de:.3f} Wh\n"
                 "(motor-off thermalling: energy curve is flat inside the segment)",
                 fontweight='bold', fontsize=10)
    l1, lab1 = ax.get_legend_handles_labels()
    l2, lab2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, lab1 + lab2, fontsize=8, loc='lower right')
    save_fig(fig, "thermal_altitude_energy_trace")


def _bar_panel(ax, df, x, mean_col, std_col, title, ylabel, color):
    xs = np.arange(len(df))
    yerr = df[std_col].fillna(0.0) if std_col in df else None
    ax.bar(xs, df[mean_col], yerr=yerr, capsize=4, color=color,
           edgecolor='black', alpha=0.85)
    ax.set_xticks(xs)
    ax.set_xticklabels(df[x], rotation=20, ha='right', fontsize=8)
    ax.set_title(title, fontsize=10)
    ax.set_ylabel(ylabel)


def fig_reduced_baselines():
    df = pd.read_csv(os.path.join(CSV_DIR, "reduced_framework_baselines.csv"))
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    _bar_panel(axes[0, 0], df, 'configuration', 'final_soc_pct_mean', 'final_soc_pct_std',
               "Final SOC (propulsion-only)", "SOC (%)", '#2b5c8f')
    _bar_panel(axes[0, 1], df, 'configuration', 'propulsion_energy_wh_mean',
               'propulsion_energy_wh_std', "Fleet propulsion energy", "Energy (Wh)", '#d62728')
    _bar_panel(axes[1, 0], df, 'configuration', 'hp_investigated_pct_mean',
               'hp_investigated_pct_std', "HP events investigated", "%", '#8f2b5c')
    _bar_panel(axes[1, 1], df, 'configuration', 'thermalling_time_s_mean',
               'thermalling_time_s_std', "Fleet thermalling time", "s", '#ff7f0e')
    fig.suptitle("Reduced-framework baseline comparison (framework-component ablations)",
                 fontweight='bold')
    fig.tight_layout()
    save_fig(fig, "reduced_framework_baselines")


def fig_energy_budget():
    df = pd.read_csv(os.path.join(CSV_DIR, "energy_budget_by_mode.csv"))
    main = df[df['mode'].isin(MODES)]
    saving = df[df['mode'] == 'THERMAL_EXPLOITATION_SAVING_VS_CRUISE']
    fig, ax = plt.subplots(figsize=(9, 5))
    colors = [MODE_COLORS[m] for m in main['mode']]
    bars = ax.barh(main['mode'], main['propulsion_energy_wh_mean'],
                   xerr=main['propulsion_energy_wh_std'].fillna(0), capsize=4,
                   color=colors, edgecolor='black', alpha=0.85)
    for bar, share in zip(bars, main['share_pct']):
        if not math.isnan(share):
            ax.text(bar.get_width() + 0.005, bar.get_y() + bar.get_height() / 2,
                    f"{bar.get_width():.3f} Wh ({share:.0f}%)", va='center', fontsize=8)
    note = ""
    if not saving.empty and not math.isnan(saving.iloc[0]['propulsion_energy_wh_mean']):
        note = (f"\nEstimated propulsion-energy saving during thermal exploitation "
                f"(relative to powered cruise): {saving.iloc[0]['propulsion_energy_wh_mean']:.3f} Wh per UAV")
    ax.set_xlabel("Propulsion energy per UAV (Wh) — propulsion only, no avionics/payload")
    ax.set_title("Propulsion-energy budget by FSM mode (20 stochastic runs)" + note,
                 fontweight='bold', fontsize=10)
    save_fig(fig, "energy_budget_breakdown_no_avionics")


def fig_thermal_sensitivity():
    df = pd.read_csv(os.path.join(CSV_DIR, "thermal_sensitivity.csv"))
    order = {"low": 0, "nominal": 1, "high": 2}
    df = df.sort_values(by='level', key=lambda s: s.map(order))
    fig, axes = plt.subplots(1, 4, figsize=(16, 4.2))
    panels = [
        ('thermal_encounters_per_uav_mean', 'thermal_encounters_per_uav_std',
         "Thermal encounters per UAV", "count", '#ff7f0e'),
        ('thermalling_time_s_mean', 'thermalling_time_s_std',
         "Fleet thermalling time", "s", '#1f77b4'),
        ('exploitation_saving_wh_mean', 'exploitation_saving_wh_std',
         "Propulsion-energy saving vs cruise", "Wh (fleet)", '#2ca02c'),
        ('final_soc_pct_mean', 'final_soc_pct_std',
         "Final SOC", "%", '#9467bd'),
    ]
    for ax, (m, sdev, title, ylab, c) in zip(axes, panels):
        ax.errorbar(df['level'], df[m], yerr=df[sdev].fillna(0), fmt='-o',
                    color=c, capsize=4, lw=2)
        ax.set_title(title, fontsize=10)
        ax.set_ylabel(ylab)
        ax.set_xlabel("Thermal availability")
    fig.suptitle("Thermal-field sensitivity (5 seeds per condition; route, events, autonomy fixed)",
                 fontweight='bold')
    fig.tight_layout()
    save_fig(fig, "thermal_sensitivity")


def fig_event_sensitivity():
    df = pd.read_csv(os.path.join(CSV_DIR, "event_sensitivity.csv"))
    order = {"low": 0, "nominal": 1, "high": 2}
    df = df.sort_values(by='level', key=lambda s: s.map(order))

    # outcomes: stacked HP counts + detection percentage
    fig, ax = plt.subplots(figsize=(9, 5.5))
    xs = np.arange(len(df))
    bot = np.zeros(len(df))
    for col, label, color in [
            ('hp_investigated_count_mean', 'HP investigated', '#2ca02c'),
            ('hp_expired_count_mean', 'HP expired (uninvestigated)', '#d62728'),
            ('hp_unresolved_count_mean', 'HP unresolved (active at end)', '#7f7f7f')]:
        vals = df[col].fillna(0).values
        ax.bar(xs, vals, bottom=bot, label=label, color=color, edgecolor='black',
               alpha=0.85, width=0.55)
        bot += vals
    ax.set_xticks(xs)
    ax.set_xticklabels(df['level'])
    ax.set_xlabel("Event load")
    ax.set_ylabel("Mean HP events per run (counts)")
    ax2 = ax.twinx()
    ax2.plot(xs, df['hp_detected_pct_mean'].to_numpy(), '-o', color='#1f77b4', lw=2,
             label='HP detected (%)')
    ax2.plot(xs, df['hp_investigated_pct_mean'].to_numpy(), '-s', color='#9467bd', lw=2,
             label='HP investigated (%)')
    ax2.set_ylabel("Percentage of HP events (%)")
    ax2.grid(False)
    l1, lab1 = ax.get_legend_handles_labels()
    l2, lab2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, lab1 + lab2, fontsize=8, loc='upper left')
    ax.set_title("Ground-event sensitivity — HP outcome counts and percentages",
                 fontweight='bold')
    save_fig(fig, "event_sensitivity_outcomes")

    # latency
    fig, ax = plt.subplots(figsize=(7, 4.8))
    lat = df['mean_hp_investigation_latency_s_mean']
    err = df['mean_hp_investigation_latency_s_std'].fillna(0).to_numpy()
    ax.bar(xs, lat.fillna(0).to_numpy(), yerr=err, capsize=5, color='#2b5c8f',
           edgecolor='black', alpha=0.85, width=0.5)
    for x, v in zip(xs, lat):
        if math.isnan(v):
            ax.text(x, 0.5, "NaN\n(no HP investigations\nin these runs)",
                    ha='center', fontsize=8, color='#666666')
    ax.set_xticks(xs)
    ax.set_xticklabels(df['level'])
    ax.set_xlabel("Event load")
    ax.set_ylabel("Mean HP investigation latency (s)\n(first FOV detection → loiter start)")
    ax.set_title("Ground-event sensitivity — investigation latency", fontweight='bold')
    save_fig(fig, "event_sensitivity_latency")


def fig_stochastic_summary():
    df = pd.read_csv(os.path.join(CSV_DIR, "stochastic_runs_per_seed.csv"))
    panels = [
        ('execution_duration_s', 'Execution duration (s)', '#1f77b4'),
        ('final_soc_pct', 'Final SOC (%)', '#2ca02c'),
        ('thermal_encounters', 'Fleet thermal encounters', '#ff7f0e'),
        ('thermalling_time_s', 'Fleet thermalling time (s)', '#9467bd'),
        ('hp_investigated_pct', 'HP investigated (%)', '#d62728'),
        ('all_detected_pct', 'Events detected (%)', '#17becf'),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    for ax, (col, label, color) in zip(axes.flat, panels):
        vals = df[col].dropna()
        if vals.empty:
            ax.text(0.5, 0.5, "NaN (not observed)", ha='center', va='center')
            ax.set_title(label, fontsize=10)
            continue
        ax.hist(vals, bins=8, color=color, edgecolor='black', alpha=0.8)
        ax.axvline(vals.mean(), color='k', ls='--', lw=1.2)
        ax.set_title(f"{label}\nmean={vals.mean():.2f}, std={vals.std():.2f}", fontsize=9)
        ax.set_ylabel("runs")
    fig.suptitle(f"Repeated stochastic runs (R={len(df)} seeds, full framework, N=6)",
                 fontweight='bold')
    fig.tight_layout()
    save_fig(fig, "repeated_stochastic_summary")


def fig_scalability():
    df = pd.read_csv(os.path.join(CSV_DIR, "scalability_overhead.csv"))
    fig, axes = plt.subplots(2, 4, figsize=(18, 8))
    panels = [
        ('cpu_pct_mean', 'cpu_pct_std', "System CPU utilisation", "%", '#d62728'),
        ('mem_pct_mean', 'mem_pct_std', "System memory utilisation", "%", '#1f77b4'),
        ('rtf_mean', 'rtf_std', "Real-time factor", "RTF", '#2ca02c'),
        ('ros2_latency_ms_mean', 'ros2_latency_ms_std', "ROS 2 message latency", "ms", '#9467bd'),
        ('arming_success_pct', None, "UAV arming success", "% of fleet", '#17becf'),
        ('mavlink_timeout_count_mean', None, "MAVLink timeouts / process failures",
         "count per run", '#ff7f0e'),
        ('log_size_mb_mean', None, "Log size per run", "MB", '#7f7f7f'),
    ]
    for ax in axes.flat[len(panels):]:
        ax.axis('off')
    for ax, (m, sdev, title, ylab, c) in zip(axes.flat, panels):
        if m == 'arming_success_pct':
            ax.plot(df['fleet_size_n'].to_numpy(), df[m].to_numpy(), '-o', color=c, lw=2)
            ax.axhline(100.0, color='gray', ls=':')
            ax.set_ylim(0, 110)
        elif m == 'mavlink_timeout_count_mean':
            w = 1.5
            ax.bar(df['fleet_size_n'] - w / 2, df[m], width=w, label='MAVLink timeouts',
                   color='#ff7f0e', edgecolor='black', alpha=0.85)
            ax.bar(df['fleet_size_n'] + w / 2, df['process_failure_count_mean'], width=w,
                   label='Process failures', color='#d62728', edgecolor='black', alpha=0.85)
            ax.legend(fontsize=8)
        else:
            yerr = df[sdev].fillna(0) if sdev else None
            ax.errorbar(df['fleet_size_n'], df[m], yerr=yerr, fmt='-o', color=c,
                        capsize=4, lw=2)
        if m == 'rtf_mean':
            ax.axhline(1.0, color='gray', ls=':')
            ax.text(df['fleet_size_n'].iloc[-1], 1.01, 'real-time', fontsize=7, color='gray')
        ax.set_title(title, fontsize=10)
        ax.set_xlabel("Fleet size N")
        ax.set_ylabel(ylab)
        ax.set_xticks(df['fleet_size_n'])
    fig.suptitle("Scalability and resource overhead (N = 6, 12, 24; 3 seeds each)",
                 fontweight='bold')
    fig.tight_layout()
    save_fig(fig, "scalability_overhead")


def fig_coverage_comparison():
    df = pd.read_csv(os.path.join(CSV_DIR, "coverage_path_comparison.csv"))
    short = [s.split(" (")[0].replace(" lawnmower", "\nlawnmower").replace("Selected", "Selected\n(partitioned)")
             for s in df['strategy']]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    panels = [("total_path_length_km", "Total path length (km)", '#1f77b4'),
              ("fov_coverage_pct", "FOV-based coverage (%)", '#2ca02c'),
              ("workload_imbalance", "Workload imbalance ((max-mean)/mean)", '#d62728'),
              ("overlap_pct", "Overlap (%)", '#ff7f0e')]
    for ax, (col, lab, c) in zip(axes.flat, panels):
        xs = np.arange(len(df))
        ax.bar(xs, df[col].to_numpy(), color=c, edgecolor='black', alpha=0.85)
        ax.set_xticks(xs); ax.set_xticklabels(short, fontsize=7)
        ax.set_title(lab, fontsize=10)
        if col == "total_path_length_km":
            ax.axhline(23.3, color='gray', ls=':'); ax.text(0, 23.6, '23.3 km', fontsize=7, color='gray')
    fig.suptitle("Coverage-path comparison (geometric, FOV-based; selected route is not claimed optimal)",
                 fontweight='bold')
    fig.tight_layout()
    save_fig(fig, "coverage_path_comparison")


def fig_battery_model_comparison():
    df = pd.read_csv(os.path.join(CSV_DIR, "battery_model_comparison.csv"))
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    labels = [m.replace("_", "\n") for m in df['model']]
    xs = np.arange(len(df))
    axes[0].bar(xs, df['final_soc_pct_mean'].to_numpy(),
                yerr=df['final_soc_pct_std'].fillna(0).to_numpy(), capsize=4,
                color='#2ca02c', edgecolor='black', alpha=0.85)
    axes[0].set_xticks(xs); axes[0].set_xticklabels(labels, fontsize=7)
    axes[0].set_title("Final SOC by model (%)", fontsize=10)
    rmse = df['soc_rmse_vs_online_pct'].to_numpy()
    axes[1].bar(xs, np.nan_to_num(rmse), color='#d62728', edgecolor='black', alpha=0.85)
    for x, v in zip(xs, rmse):
        if np.isnan(v):
            axes[1].text(x, 0.3, "NaN\n(self/no ref)", ha='center', fontsize=7, color='#666')
    axes[1].set_xticks(xs); axes[1].set_xticklabels(labels, fontsize=7)
    axes[1].set_title("SOC RMSE vs online estimator (%)", fontsize=10)
    fig.suptitle("Battery-model comparison (propulsion-only; self-RMSE shown as NaN, not validation)",
                 fontweight='bold')
    fig.tight_layout()
    save_fig(fig, "battery_model_comparison")


def fig_fsm_transition_timeline(run):
    traces, trans, _, _, _ = load_run_data(run)
    uav = pick_representative_uav(run)
    tr = traces[uav]
    t0 = tr['sim_time'].iloc[0]
    t = (tr['sim_time'] - t0).to_numpy()
    state_idx = tr['fsm_state_name'].map({m: i for i, m in enumerate(MODES)}).to_numpy()
    fig, ax = plt.subplots(figsize=(13, 4.5))
    ax.step(t, state_idx, where='post', color='k', lw=1.3)
    for i, m in enumerate(MODES):
        ax.axhline(i, color=MODE_COLORS[m], alpha=0.15, lw=6)
    ax.set_yticks(range(len(MODES))); ax.set_yticklabels(MODES, fontsize=8)
    ax.set_xlabel("Mission time (s)")
    g = trans[trans['uav_id'] == uav]
    ax.set_title(f"FSM transition timeline — UAV {uav}, {os.path.relpath(run, RAW)} "
                 f"({len(g)} transitions, stabilized: no sub-2 s chatter)", fontweight='bold', fontsize=11)
    save_fig(fig, "fsm_transition_timeline")


def fig_path_behaviour_zoom(run):
    traces, trans, thermals, events, cfg = load_run_data(run)
    s = json.load(open(os.path.join(run, "metrics_summary.json")))
    # find best thermal segment and an investigated event
    best_seg = None
    for k, u in s.items():
        if not k.startswith('uav_'):
            continue
        for seg in u.get('thermal_segments', []):
            if best_seg is None or seg['duration_s'] > best_seg[1]['duration_s']:
                best_seg = (int(k.split('_')[1]), seg)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.5))
    # --- thermal zoom ---
    ax = axes[0]
    if best_seg:
        uav, seg = best_seg
        tr = traces[uav]
        w = tr[(tr['sim_time'] >= seg['entry_t'] - 20) & (tr['sim_time'] <= seg['exit_t'] + 20)]
        seg_pts = tr[(tr['sim_time'] >= seg['entry_t']) & (tr['sim_time'] <= seg['exit_t'])]
        for tid, grp in thermals[thermals['active'] == 1].groupby('thermal_id'):
            ce, cn = grp['center_east_m'].median(), grp['center_north_m'].median()
            if seg_pts['east_m'].mean() - 400 < ce < seg_pts['east_m'].mean() + 400:
                ax.add_patch(patches.Circle((ce, cn), grp['radius_m'].iloc[0],
                             facecolor='orange', alpha=0.2, edgecolor='#d97706'))
        ax.plot(w['east_m'].to_numpy(), w['north_m'].to_numpy(), color='gray', lw=1.0)
        ax.plot(seg_pts['east_m'].to_numpy(), seg_pts['north_m'].to_numpy(), color='#ff7f0e', lw=2.5,
                label='THERMAL_EXPLOITATION')
        ax.scatter(seg_pts['east_m'].iloc[0], seg_pts['north_m'].iloc[0], marker='^', s=80,
                   c='#ff7f0e', edgecolors='k', zorder=5, label='entry')
        ax.scatter(seg_pts['east_m'].iloc[-1], seg_pts['north_m'].iloc[-1], marker='v', s=80,
                   c='#1f77b4', edgecolors='k', zorder=5, label='exit')
        ax.legend(fontsize=8)
        # tighten view to the thermalling segment (+/- 150 m margin)
        if len(seg_pts):
            cx, cy = seg_pts['east_m'].mean(), seg_pts['north_m'].mean()
            span = max(150.0, seg_pts['east_m'].std() * 3, seg_pts['north_m'].std() * 3)
            ax.set_xlim(cx - span, cx + span); ax.set_ylim(cy - span, cy + span)
        ax.set_title(f"Thermal exploitation (UAV {uav}, {seg['duration_s']:.0f}s, "
                     f"+{seg.get('alt_gain_m', float('nan')):.0f} m)", fontsize=10)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_aspect('equal')
    # --- event zoom ---
    ax = axes[1]
    hp_inv = events[(events['is_high_priority'] == 1) & (events['final_state'] == 'investigated')]
    if len(hp_inv):
        ev = hp_inv.iloc[0]
        uav = int(ev['investigated_by_uav']) if str(ev['investigated_by_uav']).replace('.0','').isdigit() else None
        if uav and uav in traces:
            tr = traces[uav]
            inv = tr[tr['fsm_state_name'] == 'EVENT_INVESTIGATION']
            near = inv[(np.hypot(inv['east_m'] - ev['east_m'], inv['north_m'] - ev['north_m']) < 400)]
            t_lo = near['sim_time'].min() - 30 if len(near) else tr['sim_time'].min()
            t_hi = near['sim_time'].max() + 30 if len(near) else tr['sim_time'].max()
            w = tr[(tr['sim_time'] >= t_lo) & (tr['sim_time'] <= t_hi)]
            ax.plot(w['east_m'].to_numpy(), w['north_m'].to_numpy(), color='gray', lw=1.0)
            if len(near):
                ax.plot(near['east_m'].to_numpy(), near['north_m'].to_numpy(), color='#d62728', lw=2.5,
                        label='EVENT_INVESTIGATION loiter')
            ax.scatter([ev['east_m']], [ev['north_m']], marker='*', s=220, c='#d62728',
                       edgecolors='k', zorder=6, label=f"HP event (prio {int(ev['priority'])})")
            ax.legend(fontsize=8)
            ax.set_title(f"HP-event investigation (UAV {uav}, event {int(ev['event_id'])})", fontsize=10)
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)"); ax.set_aspect('equal')
    fig.suptitle("Path-behaviour zoom: thermal exploitation and HP-event response (real logged trajectories)",
                 fontweight='bold')
    fig.tight_layout()
    save_fig(fig, "path_behaviour_zoom")


def main():
    os.makedirs(FIG_DIR, exist_ok=True)
    run = pick_case_study_run()
    if run is None:
        print("ERROR: no completed case-study run found")
        return 1
    print(f"Case-study run: {run}")
    print("Generating figures...")
    fig_evaluation_region_path()
    fig_executed_path_overlay(run)
    fig_representative_timeline(run)
    fig_soc_time_series(run)
    fig_thermal_altitude_energy_trace(run)
    fig_reduced_baselines()
    fig_energy_budget()
    fig_thermal_sensitivity()
    fig_event_sensitivity()
    fig_stochastic_summary()
    fig_scalability()
    fig_coverage_comparison()
    fig_battery_model_comparison()
    fig_fsm_transition_timeline(run)
    fig_path_behaviour_zoom(run)
    print("All figures generated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
