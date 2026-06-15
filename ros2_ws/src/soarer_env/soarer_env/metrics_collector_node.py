#!/usr/bin/env python3
"""Metrics collector for the Soar & Seek framework.

Writes, incrementally during the run (so a hard kill loses at most a few
seconds of data):
  framework_metrics.csv   - 1 Hz system samples (CPU, mem, RTF, ROS latency, log size)
  uav_trace_<i>.csv       - per-UAV time series (pos, alt, vel, FSM state, SOC,
                            power, wind, PX4 reference battery, wp progress)
  fsm_transitions.csv     - every FSM state transition with sim timestamp
  thermal_field.csv       - thermal footprint snapshots (1 Hz)
  detections.csv          - actual FOV event detections (not message counts)

At shutdown (and every 15 s as a crash-safe checkpoint):
  events.csv              - per-event ledger with detection/investigation outcome
  metrics_summary.json    - per-UAV and fleet-level mission metrics
  mission_metrics.csv     - flat CSV of the same

All event statistics are counts and percentages, not rates in Hz.
Timestamps are sanitised: samples that jump backwards (PX4 clock reset) are
rejected and counted, so endurance can never go negative.
"""
import os
import time
import json
import csv
import psutil
import math
import numpy as np
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from soarer_msgs.msg import (FsmState, GroundEventArray, FovDetectionArray,
                             BatteryEstimate, TelemetryExchange, VerticalWind,
                             ThermalField)
from px4_msgs.msg import BatteryStatus, VehicleLocalPosition

HP_PRIORITY_THRESHOLD = 3

STATE_NAMES = {
    FsmState.STATE_PATROL: "PATROL",
    FsmState.STATE_EVENT_INVESTIGATION: "EVENT_INVESTIGATION",
    FsmState.STATE_THERMAL_SEARCH: "THERMAL_SEARCH",
    FsmState.STATE_THERMAL_EXPLOITATION: "THERMAL_EXPLOITATION",
    FsmState.STATE_GLIDE_RETURN: "GLIDE_RETURN",
    FsmState.STATE_LANDING: "LANDING",
}


class MetricsCollectorNode(Node):
    def __init__(self):
        super().__init__('metrics_collector_node')

        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('reported_value_flag', 'JSBSIM')  # "EAGLE" or "JSBSIM"
        self.declare_parameter('num_vehicles', 2)
        self.declare_parameter('output_dir', '/home/px4_sitl/sim_paper/ros2_ws')

        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.ld_flag = self.get_parameter('reported_value_flag').value
        self.num_vehicles = self.get_parameter('num_vehicles').value
        self.output_dir = self.get_parameter('output_dir').value

        os.makedirs(self.output_dir, exist_ok=True)

        # ---- Per-UAV state -------------------------------------------------
        self.last_sim_t = {}          # uav -> last accepted sim timestamp
        self.t_anomalies = {}         # uav -> rejected timestamp count
        self.consec_rejects = {}      # uav -> consecutive rejection streak
        self.clock_resyncs = {}       # uav -> adopted clock re-syncs
        self.stale_skips = {}         # uav -> burst-reordered samples skipped
        self.uav_states = {}          # uav -> list of (t, state)
        self.uav_batteries = {}       # uav -> list of (t, soc, power_w, energy_wh)
        self.uav_kin = {}             # uav -> list of (t, n, e, alt, v)  [from PX4 directly]
        self.wp_progress = {}         # uav -> (max reached, total)
        self.state_transitions = {}   # uav -> list of (from, to, t)
        self.latest_detections = {}   # uav -> (t, [FovDetection])
        self.latest_wind = {}         # uav -> w_total_mps
        self.px4_batt = {}            # uav -> latest PX4 remaining fraction
        self.last_trace_write_t = {}  # uav -> throttle for trace rows
        self.investigations = {}      # uav -> list of dict(event_id, start_t, end_t)
        self.fsm_rejected = {}        # uav -> latest rejected-transition count
        self.fsm_reentries = {}       # uav -> latest thermal re-entry count

        # ---- Global event ledger -------------------------------------------
        self.all_events = {}          # id -> dict(priority, type, n, e, spawn, expiry, radius)
        self.first_detect = {}        # id -> (t, uav)
        self.investigated = {}        # id -> (t_start, uav)
        self.investigation_end = {}   # id -> t_complete
        self.global_sim_t = 0.0

        # ROS 2 latency samples (only from wall-clock-stamped topics)
        self.latency_samples = []

        # ---- File handles (incremental writers) -----------------------------
        self.files = {}
        self.writers = {}
        self._open_csv('framework', 'framework_metrics.csv',
                       ['wall_time', 'sim_time', 'cpu_percent', 'mem_percent',
                        'rtf_window', 'rtf_cumulative', 'avg_ros_latency_ms', 'log_size_kb'])
        self._open_csv('transitions', 'fsm_transitions.csv',
                       ['uav_id', 'sim_time', 'from_state', 'to_state'])
        self._open_csv('thermal', 'thermal_field.csv',
                       ['sim_time', 'thermal_id', 'center_north_m', 'center_east_m',
                        'radius_m', 'core_radius_m', 'w_peak_mps', 'active'])
        self._open_csv('detections', 'detections.csv',
                       ['sim_time', 'uav_id', 'event_id', 'range_m', 'confidence'])
        for i in range(1, self.num_vehicles + 1):
            self._open_csv(f'trace_{i}', f'uav_trace_{i}.csv',
                           ['sim_time', 'north_m', 'east_m', 'alt_rel_m',
                            'vx_mps', 'vy_mps', 'vz_mps', 'fsm_state', 'fsm_state_name',
                            'soc_pct', 'power_w', 'energy_consumed_wh', 'wind_w_mps',
                            'px4_batt_remaining_pct', 'wp_reached', 'wp_total'])

        # ---- Subscriptions ---------------------------------------------------
        self.create_subscription(GroundEventArray, '/soarer/events', self.events_callback, 10)
        self.create_subscription(ThermalField, '/soarer/thermals', self.thermals_callback, 10)
        self.last_thermal_log_t = -10.0

        for i in range(1, self.num_vehicles + 1):
            self.last_sim_t[i] = None
            self.t_anomalies[i] = 0
            self.consec_rejects[i] = 0
            self.clock_resyncs[i] = 0
            self.stale_skips[i] = 0
            self.uav_states[i] = []
            self.uav_batteries[i] = []
            self.uav_kin[i] = []
            self.wp_progress[i] = (0, 0)
            self.state_transitions[i] = []
            self.latest_detections[i] = (None, [])
            self.latest_wind[i] = 0.0
            self.px4_batt[i] = float('nan')
            self.last_trace_write_t[i] = -10.0
            self.investigations[i] = []
            self.fsm_rejected[i] = 0
            self.fsm_reentries[i] = 0

            self.create_subscription(
                FsmState, f'/soarer/fsm/px4_{i}',
                lambda msg, uav_id=i: self.state_callback(msg, uav_id), 10)
            self.create_subscription(
                BatteryEstimate, f'/soarer/battery/px4_{i}',
                lambda msg, uav_id=i: self.batt_callback(msg, uav_id), 10)
            self.create_subscription(
                TelemetryExchange, f'/soarer/telemetry/px4_{i}',
                lambda msg, uav_id=i: self.telemetry_callback(msg, uav_id), 10)
            self.create_subscription(
                FovDetectionArray, f'/soarer/fov/px4_{i}',
                lambda msg, uav_id=i: self.fov_callback(msg, uav_id), 10)
            self.create_subscription(
                VerticalWind, f'/soarer/wind/px4_{i}',
                lambda msg, uav_id=i: self.wind_callback(msg, uav_id), 10)
            self.create_subscription(
                BatteryStatus, f'/px4_{i}/fmu/out/battery_status',
                lambda msg, uav_id=i: self.px4_batt_callback(msg, uav_id),
                qos_profile_sensor_data)
            # Positions logged directly from PX4 (fresh, high-rate) instead of
            # the MAVSDK-relayed telemetry, which lags under multi-UAV load.
            self.create_subscription(
                VehicleLocalPosition, f'/px4_{i}/fmu/out/vehicle_local_position',
                lambda msg, uav_id=i: self.pos_callback(msg, uav_id),
                qos_profile_sensor_data)

        # Simulation/wall time tracking for RTF
        self.sim_start_time = None
        self.sim_end_time = None
        self.real_start_time = time.time()
        self.prev_sample = None  # (wall, sim)

        self.last_checkpoint_wall = time.time()

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.sample_framework)
        self.get_logger().info(
            f"Metrics Collector started for {self.num_vehicles} UAVs -> {self.output_dir}")

    # ------------------------------------------------------------------ utils
    def _open_csv(self, key, filename, headers):
        path = os.path.join(self.output_dir, filename)
        f = open(path, 'w', newline='', buffering=1)
        w = csv.writer(f)
        w.writerow(headers)
        self.files[key] = f
        self.writers[key] = w

    def stamp_to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def accept_sim_time(self, uav_id, t):
        """Sanitise per-UAV sim timestamps. Rejects backwards jumps (clock
        resets) and absurd forward jumps; returns True if t is usable.
        A sustained stream of rejections means the source clock re-synced
        (PX4 lockstep handshake): adopt the new clock and prune any samples
        recorded under the stale one."""
        last = self.last_sim_t[uav_id]
        if last is not None:
            dt = t - last
            if -5.0 <= dt < 0.0:
                # Burst-reordered stale sample (lockstep stall/race interleaving):
                # keep the freshest, skip silently. This is expected behaviour,
                # not a data anomaly.
                self.stale_skips[uav_id] += 1
                return False
            if dt < -5.0 or dt > 300.0:
                self.t_anomalies[uav_id] += 1
                self.consec_rejects[uav_id] += 1
                if self.t_anomalies[uav_id] <= 3:
                    self.get_logger().warn(
                        f"[UAV {uav_id}] rejected timestamp jump of {dt:.1f}s "
                        f"(last={last:.1f}, new={t:.1f})")
                if self.consec_rejects[uav_id] > 50:
                    self.get_logger().warn(
                        f"[UAV {uav_id}] clock re-sync detected; adopting new "
                        f"time base {t:.1f} and pruning stale samples")
                    self.clock_resyncs[uav_id] += 1
                    self.uav_states[uav_id] = [s for s in self.uav_states[uav_id] if s[0] <= t]
                    self.uav_batteries[uav_id] = [b for b in self.uav_batteries[uav_id] if b[0] <= t]
                    self.uav_kin[uav_id] = [k for k in self.uav_kin[uav_id] if k[0] <= t]
                    self.state_transitions[uav_id] = [
                        x for x in self.state_transitions[uav_id] if x[2] <= t]
                    self.last_trace_write_t[uav_id] = -10.0
                    self.consec_rejects[uav_id] = 0
                    self.last_sim_t[uav_id] = t
                    return True
                return False
        self.consec_rejects[uav_id] = 0
        self.last_sim_t[uav_id] = t
        if self.sim_start_time is None:
            self.sim_start_time = t
        if self.sim_end_time is None or t > self.sim_end_time:
            self.sim_end_time = t
        if t > self.global_sim_t:
            self.global_sim_t = t
        return True

    def measure_latency(self, stamp):
        # Only called for wall-clock-stamped topics (events, thermals, fov).
        msg_time_ns = stamp.sec * 1e9 + stamp.nanosec
        curr_time_ns = self.get_clock().now().nanoseconds
        latency_ms = (curr_time_ns - msg_time_ns) / 1e6
        if 0.0 <= latency_ms < 10000.0:
            self.latency_samples.append(latency_ms)

    # ------------------------------------------------------------- callbacks
    def events_callback(self, msg):
        self.measure_latency(msg.stamp)
        for evt in msg.events:
            if evt.id not in self.all_events:
                self.all_events[evt.id] = {
                    'priority': int(evt.priority),
                    'event_type': int(evt.event_type),
                    'north_m': float(evt.north_m),
                    'east_m': float(evt.east_m),
                    'lat_deg': float(evt.lat_deg),
                    'lon_deg': float(evt.lon_deg),
                    'spawn_time': float(evt.spawn_time),
                    'expiry_time': float(evt.expiry_time),
                    'radius_m': float(evt.radius_m),
                }

    def thermals_callback(self, msg):
        self.measure_latency(msg.stamp)
        t = float(msg.sim_time_s)
        # Log footprints at ~1 Hz of sim time
        if t - self.last_thermal_log_t >= 1.0:
            self.last_thermal_log_t = t
            for th in msg.thermals:
                self.writers['thermal'].writerow([
                    round(t, 3), th.id, round(th.center_north_m, 2),
                    round(th.center_east_m, 2), round(th.radius_m, 2),
                    round(th.core_radius_m, 2), round(th.w_peak_mps, 3),
                    int(th.active)])

    def state_callback(self, msg, uav_id):
        t = self.stamp_to_sec(msg.stamp)
        if not self.accept_sim_time(uav_id, t):
            return
        # latest FSM-reported stabilization counters
        self.fsm_rejected[uav_id] = int(msg.rejected_transitions)
        self.fsm_reentries[uav_id] = int(msg.thermal_reentries)
        prev_state = self.uav_states[uav_id][-1][1] if self.uav_states[uav_id] else None
        self.uav_states[uav_id].append((t, msg.state))

        if prev_state is not None and prev_state != msg.state:
            self.state_transitions[uav_id].append((prev_state, msg.state, t))
            self.writers['transitions'].writerow([
                uav_id, round(t, 3),
                STATE_NAMES.get(prev_state, str(prev_state)),
                STATE_NAMES.get(msg.state, str(msg.state))])

            if msg.state == FsmState.STATE_EVENT_INVESTIGATION:
                # Associate the investigation with the closest active HP
                # event in the latest FOV detection set of this UAV.
                det_t, dets = self.latest_detections[uav_id]
                best = None
                for det in dets:
                    info = self.all_events.get(det.event_id)
                    if info is None or info['priority'] < HP_PRIORITY_THRESHOLD:
                        continue
                    if best is None or det.range_m < best[1]:
                        best = (det.event_id, det.range_m)
                if best is not None:
                    evt_id = best[0]
                    if evt_id not in self.investigated:
                        self.investigated[evt_id] = (t, uav_id)
                    self.investigations[uav_id].append(
                        {'event_id': evt_id, 'start_t': t, 'end_t': None})
                else:
                    self.investigations[uav_id].append(
                        {'event_id': -1, 'start_t': t, 'end_t': None})

            if prev_state == FsmState.STATE_EVENT_INVESTIGATION:
                for inv in reversed(self.investigations[uav_id]):
                    if inv['end_t'] is None:
                        inv['end_t'] = t
                        if inv['event_id'] in self.investigated and \
                                inv['event_id'] not in self.investigation_end:
                            self.investigation_end[inv['event_id']] = t
                        break
                        break

    def batt_callback(self, msg, uav_id):
        if not self.uav_states[uav_id]:
            return
        t = self.stamp_to_sec(msg.stamp)
        if not self.accept_sim_time(uav_id, t):
            return
        self.uav_batteries[uav_id].append(
            (t, float(msg.soc_pct), float(msg.power_draw_w), float(msg.energy_consumed_wh)))

    def telemetry_callback(self, msg, uav_id):
        # Only waypoint progress is taken from the (MAVSDK-relayed) telemetry
        # exchange; kinematics come from PX4 directly via pos_callback.
        t = self.stamp_to_sec(msg.stamp)
        if not self.accept_sim_time(uav_id, t):
            return
        reached, total = self.wp_progress[uav_id]
        self.wp_progress[uav_id] = (max(reached, int(msg.current_wp_idx)),
                                    max(total, int(msg.total_wp_count)))

    def pos_callback(self, msg, uav_id):
        # Ignore boot-phase samples: until the FSM publishes (post lockstep
        # clock sync, mission start) the PX4 clock may still be re-syncing.
        if not self.uav_states[uav_id]:
            return
        t = msg.timestamp / 1e6
        if not self.accept_sim_time(uav_id, t):
            return
        alt = -float(msg.z)
        v = math.sqrt(msg.vx**2 + msg.vy**2 + msg.vz**2)

        # Trace row + kinematics record (throttled to ~2 Hz sim time)
        if t - self.last_trace_write_t[uav_id] >= 0.45:
            self.last_trace_write_t[uav_id] = t
            self.uav_kin[uav_id].append((t, float(msg.x), float(msg.y), alt, v))
            soc, power, energy = float('nan'), float('nan'), float('nan')
            if self.uav_batteries[uav_id]:
                _, soc, power, energy = self.uav_batteries[uav_id][-1]
            state = self.uav_states[uav_id][-1][1] if self.uav_states[uav_id] else 0
            reached, total = self.wp_progress[uav_id]
            self.writers[f'trace_{uav_id}'].writerow([
                round(t, 3), round(msg.x, 2), round(msg.y, 2), round(alt, 2),
                round(msg.vx, 3), round(msg.vy, 3), round(msg.vz, 3),
                int(state), STATE_NAMES.get(state, str(state)),
                round(soc, 3) if not math.isnan(soc) else 'NaN',
                round(power, 3) if not math.isnan(power) else 'NaN',
                round(energy, 5) if not math.isnan(energy) else 'NaN',
                round(self.latest_wind[uav_id], 3),
                round(self.px4_batt[uav_id] * 100.0, 2) if not math.isnan(self.px4_batt[uav_id]) else 'NaN',
                reached, total])

    def fov_callback(self, msg, uav_id):
        self.measure_latency(msg.stamp)
        # Use the sim time of the position sample the FOV node computed with,
        # so the detection log is self-consistent with the trajectory trace.
        t = msg.sim_time_s if msg.sim_time_s > 0.0 else (
            self.last_sim_t[uav_id] if self.last_sim_t[uav_id] is not None else self.global_sim_t)
        self.latest_detections[uav_id] = (t, list(msg.detections))
        for det in msg.detections:
            if det.event_id not in self.first_detect:
                self.first_detect[det.event_id] = (t, uav_id)
            self.writers['detections'].writerow([
                round(t, 3), uav_id, det.event_id,
                round(det.range_m, 2), round(det.confidence, 3)])

    def wind_callback(self, msg, uav_id):
        self.latest_wind[uav_id] = float(msg.w_total_mps)

    def px4_batt_callback(self, msg, uav_id):
        # PX4's own battery estimate (simulated battery), reference model
        self.px4_batt[uav_id] = float(msg.remaining)

    # -------------------------------------------------------- 1 Hz sampling
    def get_log_sizes(self):
        log_size_bytes = 0
        search_dirs = [
            '/home/px4_sitl/sim_paper/ros2_ws',
            '/home/px4_sitl/sim_paper/PX4-Autopilot/build/px4_sitl_default'
        ]
        for d in search_dirs:
            if not os.path.exists(d):
                continue
            for root, _, files in os.walk(d):
                if '.git' in root or 'build_jsbsim_bridge' in root:
                    continue
                for file in files:
                    if file.endswith('.log'):
                        try:
                            log_size_bytes += os.path.getsize(os.path.join(root, file))
                        except Exception:
                            pass
        return log_size_bytes / 1024.0

    def sample_framework(self):
        now = time.time()
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory().percent

        sim_now = self.sim_end_time
        rtf_window, rtf_cum = float('nan'), float('nan')
        if sim_now is not None and self.sim_start_time is not None:
            real_elapsed = now - self.real_start_time
            if real_elapsed > 1.0:
                rtf_cum = (sim_now - self.sim_start_time) / real_elapsed
            if self.prev_sample is not None:
                dw = now - self.prev_sample[0]
                ds = sim_now - self.prev_sample[1]
                if dw > 0.1:
                    rtf_window = ds / dw
            self.prev_sample = (now, sim_now)

        avg_latency = float('nan')
        if self.latency_samples:
            avg_latency = sum(self.latency_samples) / len(self.latency_samples)
            self.latency_samples = []

        self.writers['framework'].writerow([
            round(now, 3),
            round(sim_now, 3) if sim_now is not None else 'NaN',
            cpu, mem,
            round(rtf_window, 4) if not math.isnan(rtf_window) else 'NaN',
            round(rtf_cum, 4) if not math.isnan(rtf_cum) else 'NaN',
            round(avg_latency, 4) if not math.isnan(avg_latency) else 'NaN',
            round(self.get_log_sizes(), 2)])

        # Crash-safe checkpoint of the summary every 15 s
        if now - self.last_checkpoint_wall >= 15.0:
            self.last_checkpoint_wall = now
            try:
                self.save_summary(checkpoint=True)
            except Exception as e:
                self.get_logger().warn(f"Checkpoint summary failed: {e}")

    # ------------------------------------------------------------- summaries
    def thermal_segments(self, uav_id):
        """Pair THERMAL_EXPLOITATION entry/exit transitions into segments with
        altitude and propulsion-energy bookkeeping."""
        segments = []
        entry = None
        for (frm, to, t) in self.state_transitions[uav_id]:
            if to == FsmState.STATE_THERMAL_EXPLOITATION:
                entry = t
            elif frm == FsmState.STATE_THERMAL_EXPLOITATION and entry is not None:
                if t >= entry:
                    segments.append((entry, t))
                entry = None
        if entry is not None and self.sim_end_time is not None and self.sim_end_time > entry:
            segments.append((entry, self.sim_end_time))

        out = []
        tel = self.uav_kin[uav_id]
        bat = self.uav_batteries[uav_id]

        def interp(series, t, idx_val, idx_t=0):
            best = None
            for row in series:
                if row[idx_t] <= t:
                    best = row
                else:
                    break
            return best[idx_val] if best is not None else float('nan')

        for (t0, t1) in segments:
            alt0 = interp(tel, t0, 3)
            alt1 = interp(tel, t1, 3)
            e0 = interp(bat, t0, 3)
            e1 = interp(bat, t1, 3)
            out.append({
                'entry_t': t0, 'exit_t': t1, 'duration_s': t1 - t0,
                'entry_alt_m': alt0, 'exit_alt_m': alt1,
                'alt_gain_m': (alt1 - alt0) if not (math.isnan(alt0) or math.isnan(alt1)) else float('nan'),
                'energy_consumed_wh': (e1 - e0) if not (math.isnan(e0) or math.isnan(e1)) else float('nan'),
            })
        return out

    def calculate_mission_metrics(self):
        summary = {'_fleet': {}}

        sim_end = self.sim_end_time if self.sim_end_time is not None else 0.0

        # ---- Fleet-level event accounting (counts & percentages) ----------
        hp_ids = [eid for eid, e in self.all_events.items()
                  if e['priority'] >= HP_PRIORITY_THRESHOLD]
        detected_ids = set(self.first_detect.keys())
        investigated_ids = set(self.investigated.keys())

        hp_detected = [eid for eid in hp_ids if eid in detected_ids]
        hp_investigated = [eid for eid in hp_ids if eid in investigated_ids]
        # expired: lifetime ended before mission end without investigation
        hp_expired = [eid for eid in hp_ids
                      if eid not in investigated_ids
                      and self.all_events[eid]['expiry_time'] <= sim_end]
        # unresolved: still active at mission end, never investigated
        hp_unresolved = [eid for eid in hp_ids
                         if eid not in investigated_ids
                         and self.all_events[eid]['expiry_time'] > sim_end]

        latencies = []
        for eid in hp_investigated:
            if eid in self.first_detect:
                lat = self.investigated[eid][0] - self.first_detect[eid][0]
                if lat >= 0.0:
                    latencies.append(lat)

        n_total = len(self.all_events)
        n_hp = len(hp_ids)
        fleet = {
            'total_events': n_total,
            'total_hp_events': n_hp,
            'hp_detected_count': len(hp_detected),
            'hp_investigated_count': len(hp_investigated),
            'hp_expired_count': len(hp_expired),
            'hp_unresolved_count': len(hp_unresolved),
            'hp_detected_pct': 100.0 * len(hp_detected) / n_hp if n_hp else float('nan'),
            'hp_investigated_pct': 100.0 * len(hp_investigated) / n_hp if n_hp else float('nan'),
            'hp_expired_pct': 100.0 * len(hp_expired) / n_hp if n_hp else float('nan'),
            'hp_unresolved_pct': 100.0 * len(hp_unresolved) / n_hp if n_hp else float('nan'),
            'mean_hp_investigation_latency_s': float(np.mean(latencies)) if latencies else float('nan'),
            'all_detected_count': len(detected_ids),
            'all_detected_pct': 100.0 * len(detected_ids) / n_total if n_total else float('nan'),
            'sim_end_time_s': sim_end,
        }
        summary['_fleet'] = fleet

        # ---- Per-UAV metrics ----------------------------------------------
        for i in range(1, self.num_vehicles + 1):
            states = self.uav_states[i]
            batteries = self.uav_batteries[i]
            telemetry = self.uav_kin[i]   # (t, n, e, alt, v) from PX4 directly
            transitions = self.state_transitions[i]

            # Endurance from sanitised, monotonic timestamps; never negative.
            endurance = 0.0
            endurance_valid = False
            if len(telemetry) > 1:
                ts = [row[0] for row in telemetry]
                endurance = max(ts) - min(ts)
                endurance_valid = endurance >= 0.0
            endurance = max(0.0, endurance)

            final_soc = float('nan')
            if batteries:
                final_soc = batteries[-1][1]

            # Energy by FSM mode (propulsion only). Uses differences of the
            # battery node's own high-rate energy integral so mode totals sum
            # exactly to the consumed propulsion energy (no 2 Hz aliasing).
            energy_by_mode = {name: 0.0 for name in STATE_NAMES.values()}
            for idx in range(len(batteries) - 1):
                t_curr, _, _, e_curr = batteries[idx]
                t_next, _, _, e_next = batteries[idx + 1]
                de = e_next - e_curr
                if t_next <= t_curr or de < 0.0:
                    continue
                mode = "PATROL"
                for st_time, st_val in reversed(states):
                    if st_time <= t_curr:
                        mode = STATE_NAMES.get(st_val, "PATROL")
                        break
                energy_by_mode[mode] += de

            # Thermal segments
            segs = self.thermal_segments(i)
            thermal_encounters = len(segs)
            total_thermal_time = sum(max(0.0, s['duration_s']) for s in segs)
            alt_gains = [s['alt_gain_m'] for s in segs if not math.isnan(s['alt_gain_m'])]
            mean_alt_gain = float(np.mean(alt_gains)) if alt_gains else float('nan')

            # Per-UAV event work
            n_investigations = len([inv for inv in self.investigations[i] if inv['event_id'] != -1])
            uav_first_detections = sum(1 for eid, (t, u) in self.first_detect.items() if u == i)

            # Route completion from patrol waypoint progress
            route_completion_pct = float('nan')
            wp_reached, wp_total = self.wp_progress[i]
            if wp_total > 0:
                route_completion_pct = 100.0 * min(wp_reached, wp_total) / wp_total

            # Landing/termination
            entered_landing = any(st[1] == FsmState.STATE_LANDING for st in states)
            landed = False
            if entered_landing and telemetry:
                final_alt = telemetry[-1][3]
                final_v = telemetry[-1][4]
                landed = final_alt < 5.0 and final_v < 2.0
            # Safe termination = still flying at sim end OR landed
            crashed = False
            if telemetry and not entered_landing:
                final_alt = telemetry[-1][3]
                crashed = final_alt < 2.0 and endurance > 30.0
            mission_complete = (not crashed) and (
                landed or (route_completion_pct >= 100.0 if not math.isnan(route_completion_pct) else False)
                or not entered_landing)

            # FSM transition correctness
            correct_transitions = True
            for (p_st, n_st, _) in transitions:
                if p_st == FsmState.STATE_THERMAL_EXPLOITATION and n_st not in [
                        FsmState.STATE_GLIDE_RETURN, FsmState.STATE_LANDING,
                        FsmState.STATE_EVENT_INVESTIGATION, FsmState.STATE_PATROL]:
                    correct_transitions = False

            # Dynamic L/D estimate (JSBSim-coupled) vs constant Eagle value
            ld_eagle = 12.0
            ld_samples = []
            for row in telemetry:
                t_val, _, _, alt, v = row[0], row[1], row[2], row[3], row[4]
                if alt < 30.0 or v < 5.0:
                    continue
                p_draw = 0.0
                for bt in reversed(batteries):
                    if bt[0] <= t_val:
                        p_draw = bt[2]
                        break
                if p_draw > 0.1:
                    drag = (p_draw * 0.85) / v
                    ld_samples.append(14.706 / max(0.1, drag))
            ld_jsbsim = float(np.clip(np.mean(ld_samples), 3.0, 20.0)) if ld_samples else float('nan')
            reported_ld = ld_jsbsim if self.ld_flag == 'JSBSIM' else ld_eagle

            # FSM transition statistics
            n_trans = len(transitions)
            trans_per_min = (n_trans / (endurance / 60.0)) if endurance > 1.0 else float('nan')
            # per-state dwell durations
            dwell = {name: [] for name in STATE_NAMES.values()}
            for idx in range(len(states) - 1):
                t0, sv = states[idx]
                t1 = states[idx + 1][0]
                if t1 > t0:
                    dwell[STATE_NAMES.get(sv, "PATROL")].append(t1 - t0)
            all_dwells = [d for v in dwell.values() for d in v]
            min_dwell = float(np.min(all_dwells)) if all_dwells else float('nan')
            sub2s = int(sum(1 for d in all_dwells if d < 2.0))

            summary[f"uav_{i}"] = {
                "endurance_s": float(endurance),
                "endurance_valid": bool(endurance_valid),
                "timestamp_anomalies": int(self.t_anomalies[i]),
                "stale_samples_skipped": int(self.stale_skips[i]),
                "clock_resyncs": int(self.clock_resyncs[i]),
                "final_soc_pct": float(final_soc) if not math.isnan(final_soc) else None,
                "propulsion_energy_wh": energy_by_mode,
                "total_propulsion_energy_wh": float(sum(energy_by_mode.values())),
                "route_completion_pct": float(route_completion_pct) if not math.isnan(route_completion_pct) else None,
                "mission_complete": bool(mission_complete),
                "thermal_encounters": int(thermal_encounters),
                "total_thermalling_time_s": float(total_thermal_time),
                "mean_thermal_alt_gain_m": float(mean_alt_gain) if not math.isnan(mean_alt_gain) else None,
                "thermal_segments": segs,
                "events_first_detected_count": int(uav_first_detections),
                "investigations_count": int(n_investigations),
                "entered_landing": bool(entered_landing),
                "landing_success": bool(landed),
                "state_transition_correctness": bool(correct_transitions),
                "fsm_transitions": int(n_trans),
                "fsm_transitions_per_min": float(trans_per_min) if not math.isnan(trans_per_min) else None,
                "fsm_rejected_transitions": int(self.fsm_rejected[i]),
                "fsm_thermal_reentries": int(self.fsm_reentries[i]),
                "fsm_min_state_dwell_s": float(min_dwell) if not math.isnan(min_dwell) else None,
                "fsm_sub2s_dwell_count": sub2s,
                "reported_ld": float(reported_ld) if not math.isnan(reported_ld) else None,
                "raw_ld_eagle": float(ld_eagle),
                "raw_ld_jsbsim": float(ld_jsbsim) if not math.isnan(ld_jsbsim) else None,
            }

        return summary

    def save_events_csv(self):
        sim_end = self.sim_end_time if self.sim_end_time is not None else 0.0
        path = os.path.join(self.output_dir, 'events.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['event_id', 'priority', 'event_type', 'north_m', 'east_m',
                        'is_high_priority',
                        'creation_time', 'activation_time', 'expiration_time', 'radius_m',
                        'first_detect_time', 'detected_by_uav',
                        'investigation_start_time', 'investigation_complete_time',
                        'investigated_by_uav', 'final_state'])
            for eid in sorted(self.all_events.keys()):
                e = self.all_events[eid]
                det = self.first_detect.get(eid)
                inv = self.investigated.get(eid)
                inv_end = self.investigation_end.get(eid)
                if inv is not None:
                    outcome = 'investigated'
                elif e['expiry_time'] <= sim_end:
                    outcome = 'expired'
                else:
                    outcome = 'unresolved_active'
                # creation == activation in this model (events active at spawn)
                w.writerow([
                    eid, e['priority'], e['event_type'],
                    round(e['north_m'], 2), round(e['east_m'], 2),
                    int(e['priority'] >= HP_PRIORITY_THRESHOLD),
                    round(e['spawn_time'], 2), round(e['spawn_time'], 2),
                    round(e['expiry_time'], 2), round(e['radius_m'], 2),
                    round(det[0], 3) if det else 'NaN',
                    det[1] if det else 'NaN',
                    round(inv[0], 3) if inv else 'NaN',
                    round(inv_end, 3) if inv_end else 'NaN',
                    inv[1] if inv else 'NaN',
                    outcome])

    def save_summary(self, checkpoint=False):
        summary = self.calculate_mission_metrics()
        summary_path = os.path.join(self.output_dir, 'metrics_summary.json')
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=4)

        self.save_events_csv()

        # Flat per-UAV CSV (scalar fields only)
        csv_path = os.path.join(self.output_dir, 'mission_metrics.csv')
        uav_keys = [k for k in summary if k.startswith('uav_')]
        if uav_keys:
            scalar_fields = [k for k, v in summary[uav_keys[0]].items()
                             if not isinstance(v, (dict, list))]
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['uav_id'] + scalar_fields)
                for uav in uav_keys:
                    writer.writerow([uav] + [summary[uav][h] for h in scalar_fields])
        if not checkpoint:
            self.get_logger().info(f"Saved final mission metrics to {summary_path}")

    def destroy_node(self):
        try:
            self.save_summary()
        finally:
            for f in self.files.values():
                try:
                    f.close()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsCollectorNode()

    def sigterm_handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, sigterm_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
