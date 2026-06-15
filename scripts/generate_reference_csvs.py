#!/usr/bin/env python3
"""Generate static reference CSVs that do not depend on run data:
  - metric_definitions.csv         (precise definition of every reported metric)
  - uav_instance_mapping.csv       (per-UAV IDs, ports, namespaces, DDS keys)
  - framework_feature_comparison.csv (verified capabilities vs typical tools)
  - thermal_parameter_justification.csv
  - event_parameter_justification.csv
  - energy_model_parameters.csv    (aero model params; scaled vs physical battery)
"""
import os
import csv
import yaml

WS = "/home/px4_sitl/sim_paper"
CSV = os.path.join(WS, "results/csv")
os.makedirs(CSV, exist_ok=True)


def write(name, header, rows):
    with open(os.path.join(CSV, name), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)
    print(f"  {name} ({len(rows)} rows)")


def metric_definitions():
    rows = [
        ["route_completion", "%", "Fraction of a UAV's assigned patrol waypoints reached (max wp index / total wp) x100"],
        ["scenario_termination", "bool", "Run reached its time horizon or all UAVs landed/terminated without process crash"],
        ["landing_state_entry", "bool", "UAV FSM entered LANDING state (reserve-SOC or end-of-mission trigger)"],
        ["landing_completion", "bool", "UAV entered LANDING and reached <5 m AGL at <2 m/s ground speed"],
        ["mission_success", "bool", "Vehicle did not crash AND (landed OR route_completion=100% OR still airborne at horizon)"],
        ["event_detection", "event", "Active event center within camera slant range AND inside angular FOV cone (incl. footprint radius)"],
        ["event_investigation", "event", "UAV entered EVENT_INVESTIGATION targeting a detected HP event and loitered >= min_investigation_s"],
        ["event_expiration", "event", "Event lifetime ended (t >= expiration_time) before any investigation began"],
        ["unresolved_event", "event", "HP event still active at run end, never investigated"],
        ["thermal_encounter", "count", "One accepted PATROL/SEARCH/GLIDE -> THERMAL_EXPLOITATION transition (after entry confirmation)"],
        ["thermalling_duration", "s", "Cumulative wall-clock sim time a UAV spent in THERMAL_EXPLOITATION"],
        ["thermal_alt_gain", "m", "Altitude at exit minus altitude at entry of a THERMAL_EXPLOITATION segment"],
        ["propulsion_energy", "Wh", "Integral of aerodynamic propulsion power only (no avionics/payload/comms/compute)"],
        ["propulsion_energy_saving", "Wh", "Mean patrol/cruise power x thermalling time: propulsion energy avoided vs powered cruise"],
        ["final_soc", "%", "State of charge at run end, relative to the (scaled) battery capacity"],
        ["hp_detected_pct", "%", "HP events with >=1 detection / total HP events x100"],
        ["hp_investigated_pct", "%", "HP events investigated / total HP events x100"],
        ["investigation_latency", "s", "investigation_start_time - first_detect_time for investigated HP events"],
        ["fsm_transitions_per_min", "1/min", "Accepted FSM state transitions per minute of execution"],
        ["rejected_transitions", "count", "Transition requests blocked by min-dwell / cooldown guards"],
        ["real_time_factor", "ratio", "Simulated time advanced / wall time elapsed (1.0 = real time)"],
        ["uav_arming_success", "%", "UAVs that armed / UAVs requested x100 (denominator = requested fleet size)"],
    ]
    write("metric_definitions.csv", ["metric", "unit", "definition"], rows)


def uav_instance_mapping(max_n=24):
    # mirrors swarm_launcher.sh + px4-rc.mavlink + rcS after the port fix
    rows = []
    for i in range(1, max_n + 1):
        rows.append([
            i,                       # uav_id
            i,                       # px4_instance
            i + 1,                   # MAV_SYS_ID (rcS: px4_instance+1)
            i + 1,                   # UXRCE_DDS_KEY (rcS: px4_instance+1)
            f"px4_{i}",              # ROS 2 namespace
            4560 + i,                # JSBSim FDM TCP port
            14640 + i,              # MAVSDK offboard remote (FIXED band)
            14550 + i,              # GCS remote (pymavlink monitor)
            18570 + i,              # GCS local
            14580 + i,              # offboard local
            50050 + i,              # MAVSDK gRPC server port
            8889,                    # shared XRCE-DDS agent port
        ])
    write("uav_instance_mapping.csv",
          ["uav_id", "px4_instance", "mav_sys_id", "uxrce_dds_key", "ros2_namespace",
           "jsbsim_tcp_port", "mavsdk_offboard_remote_port", "gcs_remote_port",
           "gcs_local_port", "offboard_local_port", "mavsdk_grpc_port", "xrce_agent_port"],
          rows)


def feature_comparison():
    rows = [
        ["Fixed-wing flight dynamics", "Yes (JSBSim 6-DOF)", "verified: JSBSim FDM per vehicle"],
        ["PX4 SITL autopilot", "Yes", "verified: PX4 instance per vehicle"],
        ["JSBSim backend", "Yes", "verified"],
        ["ROS 2 integration", "Yes (Humble)", "verified: namespaced topics per UAV"],
        ["Concurrent multi-UAV", "Yes (tested to 24)", "verified after offboard-port fix"],
        ["Thermal field model", "Yes (Allen toroidal)", "verified: updraft injected to FDM"],
        ["Temporally activated stochastic events", "Yes (Poisson)", "verified: seeded arrivals"],
        ["FOV-based sensing", "Yes (angular + slant range)", "verified geometrically"],
        ["Propulsion-energy tracking", "Yes (aero power)", "verified: non-negative integral"],
        ["Landing / termination support", "Yes", "verified: descending landing"],
        ["Deterministic seeds", "Yes", "verified: seed -> thermal+event RNG"],
        ["Per-UAV structured logging", "Yes", "verified: traces, ledgers, manifests"],
        ["Scalability evaluation", "Yes (6/12/24)", "verified: resource overhead measured"],
        ["Compatibility with existing tools", "Yes (MAVSDK/MAVLink/DDS)", "standard interfaces"],
    ]
    write("framework_feature_comparison.csv",
          ["capability", "soar_and_seek", "evidence"], rows)


def thermal_param_justification():
    rows = [
        ["num_thermals (low/nom/high)", "4 / 8 / 16", "Spans sparse to dense soaring fields over the 9 km^2 region"],
        ["w_peak_max_mps (low/nom/high)", "3.0 / 5.0 / 6.0", "Typical low-altitude convective updraft strengths"],
        ["radius_max_m (low/nom/high)", "150 / 200 / 250", "Thermal core radii consistent with boundary-layer scales"],
        ["thermal_lifetime_s", "300 / 600 / 900", "Convective cell persistence; exponential expiry"],
        ["alt_ceiling_m", "500", "Convective boundary-layer top in the modelled scenario"],
        ["usable/entry/exit lift", "1.5 / 2.0 / 0.8 m/s", "Entry>exit hysteresis band around minimum useful lift"],
    ]
    write("thermal_parameter_justification.csv",
          ["parameter", "value", "justification"], rows)


def event_param_justification():
    rows = [
        ["event_rate_hz (low/nom/high)", "0.02 / 0.05 / 0.12", "Poisson arrival rates from sparse to busy surveillance load"],
        ["high_priority_ratio (low/nom/high)", "0.3 / 0.4 / 0.6", "Fraction of events with priority >= 3"],
        ["event_lifetime_s (low/nom/high)", "150 / 120 / 75", "Shorter active windows at higher load stress responsiveness"],
        ["event_radius_m (low/nom/high)", "25 / 20 / 15", "Ground footprint detectable within sensor swath"],
        ["initial_event_count", "6 / 12 / 24", "Seed population so the region is active from t0"],
        ["priority_threshold", "3", "Events with priority >= 3 trigger investigation"],
    ]
    write("event_parameter_justification.csv",
          ["parameter", "value", "justification"], rows)


def energy_model_parameters():
    cfg = yaml.safe_load(open(os.path.join(WS, "configs/scenario_nominal.yaml")))
    b = cfg['battery_estimator_node']['ros__parameters']
    rows = [
        ["mass_kg", b.get("mass_kg", 1.5), "Airframe mass for weight/lift"],
        ["wing_area_m2", b.get("wing_area_m2", 0.9), "Reference wing area"],
        ["aspect_ratio", b.get("AR", 14.0), "Wing aspect ratio"],
        ["CD0", b.get("CD0", 0.012), "Zero-lift drag coefficient"],
        ["oswald_e", b.get("oswald_e", 0.95), "Oswald efficiency"],
        ["eta_prop", b.get("eta_prop", 0.85), "Propulsive efficiency"],
        ["rho_kg_m3", b.get("rho_kg_m3", 1.225), "Air density"],
        ["V_nom_v", b.get("V_nom", 14.8), "Nominal battery voltage"],
        ["battery_capacity_scaled_wh", b.get("battery_capacity_wh"),
         "SCALED capacity used to TRIGGER FSM energy states within the short horizon"],
        ["battery_capacity_physical_wh", 76.96,
         "PHYSICAL 14.8V 5.2Ah pack used for ENERGY-MODEL assessment (power/energy are physical)"],
        ["energy_scope", "propulsion_only",
         "Excludes avionics, payload, communication, onboard computing"],
    ]
    write("energy_model_parameters.csv", ["parameter", "value", "notes"], rows)


def main():
    print("Generating reference CSVs...")
    metric_definitions()
    uav_instance_mapping()
    feature_comparison()
    thermal_param_justification()
    event_param_justification()
    energy_model_parameters()
    print("Reference CSVs done.")


if __name__ == "__main__":
    main()
