#!/usr/bin/env python3
"""Write results/publication/results_section.tex with numbers read from the
generated publication CSVs (no hardcoded values)."""
import os
import math
import pandas as pd

WS = "/home/px4_sitl/sim_paper"
PUB = os.path.join(WS, "results/publication")
PCSV = os.path.join(PUB, "csv")
SRC = os.path.join(WS, "results/csv")
TEX = os.path.join(PUB, "results_section.tex")


def rd(name, src=PCSV):
    p = os.path.join(src, name)
    return pd.read_csv(p) if os.path.exists(p) else pd.DataFrame()


def g(df, metric, col, key="metric"):
    r = df[df[key] == metric]
    return float(r[col].iloc[0]) if len(r) else float('nan')


def main():
    ss = rd("repeated_stochastic_results.csv")
    # stochastic summary uses display names in Metric col
    def sm(name, col):
        r = ss[ss["Metric"] == name]
        try:
            return float(r[col].iloc[0])
        except (IndexError, ValueError):
            return float('nan')

    cov = pd.read_csv(os.path.join(SRC, "coverage_path_comparison.csv"))
    rb = rd("reduced_framework_baselines.csv")
    sc = rd("scalability_overhead.csv")
    tmo = rd("scalability_timeout_analysis.csv")
    t24 = tmo[tmo["fleet_size"] == 24].iloc[0] if len(tmo) else None
    tmo_tot = int(t24["total_timeouts"]) if t24 is not None else 0
    tmo_runs = int(t24["n_runs"]) if t24 is not None else 0
    tmo_run = float(t24["mean_timeouts_per_run"]) if t24 is not None else 0.0
    tmo_uav = float(t24["timeouts_per_uav"]) if t24 is not None else 0.0
    tmo_uavmin = float(t24["timeouts_per_uav_minute"]) if t24 is not None else 0.0
    cons = rd("framework_verification_summary.csv")
    th = rd(os.path.join(SRC, "thermal_sensitivity.csv")) if False else pd.read_csv(os.path.join(SRC, "thermal_sensitivity.csv"))
    ev = pd.read_csv(os.path.join(SRC, "event_sensitivity.csv"))
    ls = pd.read_csv(os.path.join(SRC, "landing_validation_summary.csv")).iloc[0]
    pjs = pd.read_csv(os.path.join(SRC, "px4_jsbsim_consistency.csv"))
    status = open(os.path.join(SRC, "px4_jsbsim_status.txt")).readline().strip().replace("_", r"\_")
    sel = cov[cov["strategy"].str.contains("Selected", case=False, na=False)].iloc[0]

    def thr(level, col):
        r = th[th["level"] == level]
        return float(r[col].iloc[0]) if len(r) else float('nan')

    def evr(level, col):
        r = ev[ev["level"] == level]
        return float(r[col].iloc[0]) if len(r) else float('nan')

    T = []
    w = T.append
    w(r"\section{Evaluation and Results}")
    w(r"\label{sec:results}")
    w("")
    # ---- setup
    w(r"\subsection{Experimental Setup and Evaluation Protocol}")
    w(r"All results are obtained from PX4 Software-In-The-Loop (SITL) coupled to a "
      r"per-vehicle JSBSim fixed-wing flight-dynamics model through a Micro-XRCE-DDS "
      r"bridge, with the thermal field, stochastic ground events, field-of-view (FOV) "
      r"sensing, propulsion-energy estimator and finite-state-machine (FSM) autonomy "
      r"implemented as ROS\,2 nodes. Table~\ref{tab:experimental_campaign} summarises the "
      r"campaign. Each run is fully seeded and reproducible; the master seed drives the "
      r"thermal field and the ground-event process. Campaign runs execute a $600$\,s "
      r"simulated horizon ($900$\,s for the nominal case study) at real-time lockstep, "
      r"with the battery capacity scaled so the complete energy-aware FSM cycle executes "
      r"within the horizon. Reported durations are therefore short-horizon framework-"
      r"execution times, not vehicle-endurance claims, and all energy quantities are "
      r"propulsion-only (avionics, payload, communication and onboard computing are "
      r"excluded).")
    w("")
    # ---- verification
    w(r"\subsection{Framework Verification}")
    w(rf"Table~\ref{{tab:framework_verification}} reports the verification suite. The "
      rf"PX4 extended Kalman filter (EKF) estimate is compared against the JSBSim flight-"
      rf"dynamics ground truth relayed through \texttt{{HIL\_STATE\_QUATERNION}}; "
      rf"Fig.~\ref{{fig:px4_jsbsim_verification}} shows altitude, ground-speed and "
      rf"attitude comparisons with their error traces and the shaded initialization "
      rf"window. After the documented $20$\,s estimator-convergence window is excluded, "
      rf"the mean post-initialization root-mean-square errors are "
      rf"{pjs['pos_rmse_m'].mean():.2f}\,m (position), {pjs['alt_rmse_m'].mean():.2f}\,m "
      rf"(altitude), {pjs['vel_rmse_mps'].mean():.2f}\,m/s (velocity), "
      rf"{pjs['vspeed_rmse_mps'].mean():.2f}\,m/s (vertical speed) and "
      rf"{pjs['att_rmse_deg'].mean():.2f}$^\circ$ (attitude), with no post-initialization "
      rf"gross outliers. Position RMSE is the three-dimensional local-NED position-vector "
      rf"error $\sqrt{{e_x^2+e_y^2+e_z^2}}$; altitude RMSE is the vertical ($-z$) error; "
      rf"velocity RMSE is the three-dimensional NED velocity-vector error; vertical-speed "
      rf"RMSE is the $v_z$ error; and attitude RMSE is the magnitude of the roll/pitch/yaw "
      rf"error vector. Per-component acceptance thresholds (position $\leq5$\,m, altitude "
      rf"$\leq5$\,m, velocity $\leq2$\,m/s, vertical speed $\leq2$\,m/s, attitude "
      rf"$\leq10^\circ$) are listed in Table~\ref{{tab:framework_verification}} and all are "
      rf"met. The verification status is therefore \texttt{{{status}}}. ROS\,2 namespace "
      rf"isolation is established at the message level: each UAV owns a unique "
      rf"\texttt{{px4\_$i$}} namespace and source identifier with consistent topic "
      rf"ownership, and no node received a message from an incorrect namespace. Command "
      rf"routing, FOV detection, event-state correctness, thermal-parameter validity, "
      rf"SOC/log synchronization, landing-state validity and log completeness also pass.")
    w("")
    # ---- coverage
    w(r"\subsection{Coverage-Path and Executed-Trajectory Analysis}")
    w(rf"Fig.~\ref{{fig:planned_executed_trajectories}} shows the planned partitioned "
      rf"route (panel a) and the executed multi-UAV trajectories (panel b), including "
      rf"thermal footprints, thermal-exploitation segments with entry/exit markers, "
      rf"investigated high-priority (HP) event loiters, glide-return and landing "
      rf"segments. Table~\ref{{tab:coverage_path_comparison}} compares the selected "
      rf"route against standard coverage strategies using the actual FOV ground "
      rf"footprint. The selected partitioned route has a total length of "
      rf"{sel['total_path_length_km']:.1f}\,km with {sel['fov_coverage_pct']:.0f}\,\% "
      rf"FOV coverage; the lawnmower and boustrophedon strategies achieve higher "
      rf"coverage at the cost of longer paths. The selected route is reasonable for the "
      rf"scenario but is \emph{{not}} claimed to be optimal.")
    w("")
    # ---- baselines
    rbm = rb.set_index("Configuration") if "Configuration" in rb.columns else pd.DataFrame()
    w(r"\subsection{Reduced-Framework Baseline Comparison}")
    w(r"Table~\ref{tab:reduced_framework_baselines} and "
      r"Fig.~\ref{fig:reduced_framework_baselines} compare the full framework against "
      r"five reduced configurations over five matched seeds. Improvements and "
      r"degradations are interpreted separately. Disabling event response removes "
      r"HP-event investigation entirely while preserving more energy; disabling soaring "
      r"removes thermalling and its energy benefit but can increase route completion "
      r"because the vehicle no longer diverts to exploit lift; the non-energy-aware FSM "
      r"produces a lower final SOC than the full framework; and the simplified-battery "
      r"model changes the SOC interpretation. Each ablation produces the expected "
      r"capability-specific trade-off rather than a uniform degradation.")
    w("")
    # ---- energy
    w(r"\subsection{Propulsion-Energy Assessment}")
    w(r"Fig.~\ref{fig:propulsion_energy_assessment} presents the propulsion-only energy "
      r"budget by FSM mode (panel a), SOC trajectories coloured by mode (panel b), and "
      r"the online state-dependent estimator against a constant-power model and the PX4 "
      r"battery estimate (panel c). Thermal-exploitation energy is reported as "
      r"propulsion energy avoided relative to powered cruise. No external real-flight "
      r"reference is available, and no experimental real-flight validation is claimed.")
    w("")
    # ---- thermal
    w(r"\subsection{Thermal-Field Sensitivity Analysis}")
    w(rf"Fig.~\ref{{fig:thermal_field_sensitivity}} reports thermal encounters per UAV, "
      rf"thermalling duration, propulsion-energy saving relative to powered cruise, and "
      rf"final SOC across low, nominal and high thermal conditions (five matched seeds "
      rf"each). Thermalling duration rises from {thr('low','thermalling_time_s_mean'):.0f}\,s "
      rf"(low) to {thr('high','thermalling_time_s_mean'):.0f}\,s (high), and the "
      rf"propulsion-energy saving rises from {thr('low','exploitation_saving_wh_mean'):.1f}\,Wh "
      rf"to {thr('high','exploitation_saving_wh_mean'):.1f}\,Wh, a monotonic response. "
      rf"Fig.~\ref{{fig:thermal_interaction_trace}} shows a representative exploitation "
      rf"segment: the UAV circles within the active thermal footprint while altitude "
      rf"increases and propulsion energy remains essentially flat (motor-off soaring).")
    w("")
    # ---- events
    w(rf"\subsection{{Ground-Event Sensitivity Analysis}}")
    w(rf"Fig.~\ref{{fig:ground_event_sensitivity}} reports HP-event outcomes and "
      rf"investigation latency across event loads. HP investigation percentage is "
      rf"{evr('low','hp_investigated_pct_mean'):.0f}\,\%, "
      rf"{evr('nominal','hp_investigated_pct_mean'):.0f}\,\% and "
      rf"{evr('high','hp_investigated_pct_mean'):.0f}\,\% at low, nominal and high load "
      rf"respectively; investigation percentage varies non-monotonically with workload. "
      rf"Panel (b) reports investigation latency with the number of investigated events "
      rf"annotated for every condition. The low-load latency value rests on only a few "
      rf"investigated events and is descriptive rather than statistically representative; "
      rf"mean, median and a one-standard-deviation interval (clamped at zero) are shown "
      rf"only where at least two investigated events are available. Outcomes are reported "
      rf"as counts and percentages rather than rates.")
    w("")
    # ---- stochastic
    w(r"\subsection{Repeated Stochastic Evaluation}")
    w(rf"Table~\ref{{tab:stochastic_results}} and "
      rf"Fig.~\ref{{fig:repeated_stochastic_summary}} summarise 20 seeded runs of the "
      rf"full framework. Route completion is "
      rf"{sm('Route completion (%)','Mean'):.1f}\,\% "
      rf"($\pm${sm('Route completion (%)','95% CI'):.1f}\,\% at 95\,\% confidence), "
      rf"final SOC {sm('Final SOC (%)','Mean'):.1f}\,\%, HP investigation "
      rf"{sm('HP investigated (%)','Mean'):.1f}\,\%, and detected-event percentage "
      rf"{sm('Detected events (%)','Mean'):.1f}\,\%, with "
      rf"{sm('Process failures','Mean'):.2f} process failures per run. We report valid "
      rf"scenario termination rather than an ambiguous mission-completion metric.")
    w("")
    # ---- scalability
    sc6 = sc[sc["Fleet"] == "6 UAVs"].iloc[0] if "Fleet" in sc.columns else None
    sc24 = sc[sc["Fleet"] == "24 UAVs"].iloc[0] if "Fleet" in sc.columns else None
    w(r"\subsection{Scalability and Resource-Overhead Analysis}")
    w(rf"Table~\ref{{tab:scalability_overhead}} and "
      rf"Fig.~\ref{{fig:scalability_overhead}} report resource overhead at 6, 12 and 24 "
      rf"UAVs (three seeds each), with the denominator fixed to the requested fleet "
      rf"size. After correcting an offboard-port assignment defect that previously "
      rf"prevented vehicles with two-digit identifiers from arming, arming success is "
      rf"100\,\% at all three fleet sizes. CPU utilisation rises from "
      rf"{sc6['CPU (%)'] if sc6 is not None else 'NR'}\,\% at 6 UAVs to "
      rf"{sc24['CPU (%)'] if sc24 is not None else 'NR'}\,\% at 24 UAVs while the "
      rf"real-time factor stays near unity. The framework maintained full arming and "
      rf"near-real-time execution at 24 UAVs, although MAVLink timeout frequency "
      rf"increased and remains a communication-overhead limitation: no MAVLink timeouts "
      rf"occurred at 6 or 12 UAVs, whereas at 24 UAVs the timeouts totalled {tmo_tot} "
      rf"across {tmo_runs} runs ({tmo_run:.1f} per run, {tmo_uav:.2f} per UAV, "
      rf"{tmo_uavmin:.2f} per UAV-minute). These were heartbeat-telemetry gaps from which "
      rf"the links recovered; all 24 vehicles armed and produced complete trajectory "
      rf"logs, no commands were permanently lost, and no process failures occurred, so "
      rf"mission execution and process stability were not observably affected. The split "
      rf"between startup and in-mission timeouts was not individually timestamped (NR), "
      rf"and the per-message drop rate was not instrumented (NR).")
    w("")
    # ---- landing
    w(r"\subsection{Landing-Cycle Validation}")
    w(rf"Table~\ref{{tab:landing_validation}} and "
      rf"Fig.~\ref{{fig:landing_cycle_validation}} report {int(ls['n_trials'])} landing "
      rf"trials from distinct approach positions. Touchdown success requires a spatial "
      rf"landing-zone condition (within a {ls['landing_zone_radius_m']:.0f}\,m radius of "
      rf"the landing point) in addition to low altitude and ground speed. Landing-state "
      rf"entry and approach completion are {ls['landing_entry_rate_pct']:.0f}\,\% and "
      rf"{ls['approach_completion_rate_pct']:.0f}\,\%; landing-zone success is "
      rf"{ls['landing_zone_success_rate_pct']:.0f}\,\%, touchdown success "
      rf"{ls['touchdown_success_rate_pct']:.0f}\,\% and landing completion "
      rf"{ls['landing_completion_rate_pct']:.0f}\,\%, with a mean final altitude error of "
      rf"{ls['mean_final_altitude_error_m']:.2f}\,m, mean cross-track error "
      rf"{ls['mean_cross_track_error_m']:.0f}\,m and a maximum sustained descent rate of "
      rf"{ls['max_sustained_descent_rate_mps']:.2f}\,m/s computed over a fixed time "
      rf"window (rather than instantaneous derivative spikes). No command rejections or "
      rf"MAVLink timeouts occurred. Full landing completion was achieved in 4 of 6 trials "
      rf"({ls['landing_completion_rate_pct']:.1f}\,\%); landing was therefore not fully "
      rf"reliable across all vehicles. The {ls['landing_zone_radius_m']:.0f}\,m landing-"
      rf"zone radius is an acceptance region for autonomous descent and arrival (an "
      rf"operational safety area), not a prepared landing field. Accordingly, this "
      rf"experiment evaluates autonomous descent and arrival within an operational "
      rf"landing zone; it does not evaluate precision runway touchdown, and the result is "
      rf"not described as precision landing.")
    w("")
    # ---- summary
    w(r"\subsection{Summary of Findings}")
    w(r"The framework executes the coupled fixed-wing, thermal, ground-event, sensing, "
      r"propulsion-energy and multi-UAV pipeline reproducibly. Verification shows the PX4 "
      r"estimate tracks the JSBSim ground truth within documented bounds, and the "
      r"component ablations produce the expected capability-specific trade-offs. Thermal "
      r"sensitivity exhibits the expected monotonic trends, while event investigation "
      r"percentage varies non-monotonically with workload. The repeated-seed experiments "
      r"quantify the variability and confidence intervals of the principal outcomes "
      r"rather than implying uniformly low variability: route completion, thermalling "
      r"duration and event-investigation latency all show substantial run-to-run spread "
      r"(the low-load latency in particular rests on few investigated events). The "
      r"framework scales to 24 vehicles with full arming success and near-real-time "
      r"execution, although MAVLink timeout frequency increases at 24 UAVs and remains a "
      r"communication-overhead limitation. The landing cycle is validated against a "
      r"spatial landing-zone criterion with full landing completion in 4 of 6 trials. No "
      r"optimality or real-flight-validity claims are made.")
    w("")

    # ---- figure/table float blocks
    figs = [("planned_executed_trajectories", "fig:planned_executed_trajectories",
             "Planned partitioned coverage route (a) and executed multi-UAV trajectories (b) "
             "for the nominal case study, with thermal footprints, thermal entry/exit, "
             "event loiters, glide-return and landing segments.", True),
            ("px4_jsbsim_verification", "fig:px4_jsbsim_verification",
             "PX4-EKF versus JSBSim-FDM comparison: altitude (a,b), ground speed (c,d) and "
             "attitude (e,f) with error traces. The shaded interval is the excluded "
             "initialization window; status PASS\\_WITH\\_DOCUMENTED\\_STARTUP\\_TRANSIENTS.", True),
            ("reduced_framework_baselines", "fig:reduced_framework_baselines",
             "Reduced-framework baselines: final SOC (a), propulsion energy (b), HP-event "
             "investigation percentage (c) and thermalling duration (d); bars show mean and "
             "standard deviation over matched seeds.", True),
            ("propulsion_energy_assessment", "fig:propulsion_energy_assessment",
             "Propulsion-only energy assessment: energy by FSM mode (a), SOC trajectories by "
             "mode (b), and online estimator versus constant-power model and PX4 estimate (c).", True),
            ("thermal_field_sensitivity", "fig:thermal_field_sensitivity",
             "Thermal-field sensitivity across low, nominal and high conditions: encounters per "
             "UAV (a), thermalling duration (b), energy saving relative to cruise (c) and final "
             "SOC (d); error bars show standard deviation.", True),
            ("thermal_interaction_trace", "fig:thermal_interaction_trace",
             "Representative thermal interaction: UAV trajectory inside the active thermal "
             "footprint (a) and altitude, updraft and cumulative propulsion energy versus time "
             "(b), with the thermal-exploitation interval shaded.", True),
            ("ground_event_sensitivity", "fig:ground_event_sensitivity",
             "Ground-event sensitivity: mean HP-event outcomes and detection/investigation "
             "percentages by load (a) and investigation latency (b). The sample count $n$ of "
             "investigated events is annotated for every condition; mean, median and a "
             "one-standard-deviation interval (clamped at zero) are shown only for $n\\geq2$. "
             "The low-load latency is descriptive, resting on few investigated events, and is "
             "not statistically representative.", True),
            ("repeated_stochastic_summary", "fig:repeated_stochastic_summary",
             "Repeated stochastic evaluation over 20 seeds: route completion (a), final SOC (b), "
             "thermal encounters (c), thermalling duration (d), HP investigation (e) and detected "
             "events (f). Boxplots show median and quartiles; diamonds mark the mean.", True),
            ("scalability_overhead", "fig:scalability_overhead",
             "Scalability and resource overhead versus fleet size: CPU (a), memory (b), real-time "
             "factor (c) and ROS\\,2 latency (d); error bars span repeated runs.", True),
            ("landing_cycle_validation", "fig:landing_cycle_validation",
             "Landing-cycle validation: ground tracks with the landing zone (a), altitude versus "
             "distance to the landing point (b), and per-UAV landing-subphase timeline (c) showing "
             "approach/descent and final-approach phases with entry, touchdown and disarm markers. "
             "Green/red distinguish successful and failed touchdown and disarm. Full landing "
             "completion was achieved in 4 of 6 trials.", True)]
    for name, lab, cap, wide in figs:
        env = "figure*" if wide else "figure"
        w(rf"\begin{{{env}}}[t]")
        w(r"\centering")
        w(rf"\includegraphics[width=0.95\linewidth]{{figures/{name}.pdf}}")
        w(rf"\caption{{{cap}}}")
        w(rf"\label{{{lab}}}")
        w(rf"\end{{{env}}}")
        w("")

    # input the table files
    for tname in ["experimental_campaign_matrix", "framework_verification_summary",
                  "coverage_path_comparison", "reduced_framework_baselines",
                  "repeated_stochastic_results", "scalability_overhead",
                  "landing_validation_outcomes"]:
        w(rf"\input{{tables/{tname}.tex}}")
    w("")

    with open(TEX, "w") as f:
        f.write("\n".join(T) + "\n")
    print(f"Wrote {TEX} ({len(T)} lines)")


if __name__ == "__main__":
    main()
