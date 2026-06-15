#!/usr/bin/env python3
import os
import sys
import time
import math
import asyncio
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# MAVSDK
from mavsdk import System
from mavsdk.offboard import Attitude
from mavsdk.mission import MissionItem, MissionPlan
from mavsdk.action import OrbitYawBehavior

# Custom messages
from soarer_msgs.msg import FsmState, ThermalCue, TelemetryExchange, GroundEventArray, FovDetectionArray, VerticalWind, BatteryEstimate, ThermalField
from px4_msgs.msg import VehicleLocalPosition

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')

        # Declare parameters
        self.declare_parameter('vehicle_id', 1)
        self.declare_parameter('update_rate_hz', 2.0)
        self.declare_parameter('reserve_soc', 7.0)
        self.declare_parameter('search_soc', 30.0)
        self.declare_parameter('usable_lift_threshold', 1.5)
        self.declare_parameter('thermal_ceiling_m', 150.0)
        self.declare_parameter('event_investigation_priority_threshold', 3)
        self.declare_parameter('investigation_duration_s', 30.0)
        self.declare_parameter('max_cue_distance_m', 1000.0)
        self.declare_parameter('enable_soaring', True)
        self.declare_parameter('enable_event_investigation', True)
        # --- FSM stabilization (hysteresis + timing guards) ---------------
        # Separate thermal entry/exit lift thresholds (hysteresis band) so a
        # UAV does not chatter in/out of THERMAL_EXPLOITATION near one value.
        self.declare_parameter('thermal_entry_lift_mps', 2.0)   # must exceed to ENTER
        self.declare_parameter('thermal_exit_lift_mps', 0.8)    # drop below to EXIT
        self.declare_parameter('thermal_entry_confirm_s', 2.0)  # sustained lift before entry
        self.declare_parameter('thermal_loss_timeout_s', 5.0)   # grace before exit on lift loss
        self.declare_parameter('min_state_dwell_s', 5.0)        # min time in a state
        self.declare_parameter('transition_cooldown_s', 3.0)    # min gap between transitions
        self.declare_parameter('min_investigation_s', 15.0)     # min loiter before leaving
        self.declare_parameter('reinvestigation_cooldown_s', 120.0)  # don't re-loiter same event

        # Flat arrays for waypoints (N, E, Alt, N, E, Alt, ...)
        for i in range(1, 31):
            self.declare_parameter(f'patrol_waypoints_{i}', [200.0, 200.0, 60.0, 200.0, -200.0, 60.0, -200.0, -200.0, 60.0, -200.0, 200.0, 60.0])

        # Get parameter values
        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.reserve_soc = self.get_parameter('reserve_soc').value
        self.search_soc = self.get_parameter('search_soc').value
        self.usable_lift_threshold = self.get_parameter('usable_lift_threshold').value
        self.thermal_ceiling_m = self.get_parameter('thermal_ceiling_m').value
        self.event_priority_thresh = self.get_parameter('event_investigation_priority_threshold').value
        self.investigation_duration = self.get_parameter('investigation_duration_s').value
        self.max_cue_distance = self.get_parameter('max_cue_distance_m').value
        self.enable_soaring = self.get_parameter('enable_soaring').value
        self.enable_event_investigation = self.get_parameter('enable_event_investigation').value
        self.thermal_entry_lift = self.get_parameter('thermal_entry_lift_mps').value
        self.thermal_exit_lift = self.get_parameter('thermal_exit_lift_mps').value
        self.thermal_entry_confirm_s = self.get_parameter('thermal_entry_confirm_s').value
        self.thermal_loss_timeout_s = self.get_parameter('thermal_loss_timeout_s').value
        self.min_state_dwell_s = self.get_parameter('min_state_dwell_s').value
        self.transition_cooldown_s = self.get_parameter('transition_cooldown_s').value
        self.min_investigation_s = self.get_parameter('min_investigation_s').value
        self.reinvestigation_cooldown_s = self.get_parameter('reinvestigation_cooldown_s').value
        self.investigated_event_times = {}   # event_id -> sim time investigation ended
        # transition bookkeeping
        self.state_entry_time = 0.0          # sim time current state was entered
        self.last_transition_time = -1e9     # sim time of last accepted transition
        self.rejected_transitions = 0        # transitions blocked by dwell/cooldown
        self.lift_above_since = None         # when sustained entry-lift began
        self.thermal_reentries = 0           # GLIDE/PATROL -> EXPLOITATION re-entries

        # Fetch patrol waypoints
        wp_param_name = f'patrol_waypoints_{self.vehicle_id}'
        if not self.has_parameter(wp_param_name):
            wp_param_name = 'patrol_waypoints_1'

        wps_flat = self.get_parameter(wp_param_name).value
        self.patrol_wps = []
        for j in range(0, len(wps_flat), 3):
            if j + 2 < len(wps_flat):
                self.patrol_wps.append((wps_flat[j], wps_flat[j+1], wps_flat[j+2]))
        # Mission altitude for investigation orbits / search goto follows the route
        self.mission_alt = float(self.patrol_wps[0][2]) if self.patrol_wps else 60.0

        # FSM State
        self.state = FsmState.STATE_PATROL

        # Telemetry and states
        self.uav_lat = 0.0
        self.uav_lon = 0.0
        self.uav_alt_rel = 0.0
        self.uav_n = 0.0
        self.uav_e = 0.0
        self.uav_d = 0.0
        self.uav_vx = 0.0
        self.uav_vy = 0.0
        self.uav_vz = 0.0
        self.uav_heading = 0.0
        self.uav_in_air = False

        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        self.current_wp_idx = 0
        self.total_wp_count = 0

        self.current_soc = 100.0
        self.current_wind = 0.0
        self.fov_detections = []
        self.active_events = {}
        self.active_thermals = []
        self.thermal_cues = []

        # Offboard targets
        self.target_roll = 0.0
        self.target_pitch = 2.0
        self.target_yaw = 0.0
        self.target_thrust = 0.0
        self.offboard_active = False

        # Target event for INVESTIGATION
        self.target_event = None
        self.investigation_start_time = 0.0

        # Search targets
        self.search_target_n = 0.0
        self.search_target_e = 0.0
        self.search_start_time = 0.0
        self.search_timeout_s = 180.0

        # Active exploiting thermal details
        self.last_usable_lift_time = 0.0

        # Simulation time tracking
        self.t_sim = 0.0
        self.has_sim_time = False
        self.last_fsm_tick_time = 0.0

        # Subscriptions
        self.events_sub = self.create_subscription(GroundEventArray, '/soarer/events', self.events_callback, 10)
        self.thermals_sub = self.create_subscription(ThermalField, '/soarer/thermals', self.thermals_callback, 10)
        self.fov_sub = self.create_subscription(FovDetectionArray, f'/soarer/fov/px4_{self.vehicle_id}', self.fov_callback, 10)
        self.batt_sub = self.create_subscription(BatteryEstimate, f'/soarer/battery/px4_{self.vehicle_id}', self.batt_callback, 10)
        self.wind_sub = self.create_subscription(VerticalWind, f'/soarer/wind/px4_{self.vehicle_id}', self.wind_callback, 10)
        self.cue_sub = self.create_subscription(ThermalCue, '/soarer/thermal_cues', self.cue_callback, 10)
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/px4_{self.vehicle_id}/fmu/out/vehicle_local_position',
            self.pos_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self.fsm_pub = self.create_publisher(FsmState, f'/soarer/fsm/px4_{self.vehicle_id}', 10)
        self.telemetry_pub = self.create_publisher(TelemetryExchange, f'/soarer/telemetry/px4_{self.vehicle_id}', 10)
        self.cue_pub = self.create_publisher(ThermalCue, '/soarer/thermal_cues', 10)

        # Offset gRPC port to prevent conflicts when running multiple FSM instances on the same host
        self.drone = System(port=50050 + self.vehicle_id)

    def events_callback(self, msg):
        self.active_events = {e.id: e for e in msg.events}

    def thermals_callback(self, msg):
        self.active_thermals = msg.thermals

    def fov_callback(self, msg):
        self.fov_detections = msg.detections

    def batt_callback(self, msg):
        self.current_soc = msg.soc_pct

    def wind_callback(self, msg):
        self.current_wind = msg.w_total_mps

    def cue_callback(self, msg):
        if msg.vehicle_id != self.vehicle_id:
            msg_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9
            self.thermal_cues.append((msg_time, msg.north_m, msg.east_m, msg.latitude_deg, msg.longitude_deg))

    def pos_callback(self, msg):
        current_time = msg.timestamp / 1e6
        if self.has_sim_time:
            dt = current_time - self.t_sim
            if dt <= 0.0:
                # Out-of-order sample (best-effort QoS reordering): keep the
                # monotonic clock, never adopt a backwards timestamp.
                return
            if dt > 60.0:
                self.get_logger().info(f"Clock jump of {dt:.3f}s detected in FSM. Adjusting FSM timers.")
                self.investigation_start_time += dt
                self.search_start_time += dt
                self.last_usable_lift_time += dt
                self.last_fsm_tick_time += dt
                if hasattr(self, 'last_cue_publish_time'):
                    self.last_cue_publish_time += dt
        self.t_sim = current_time
        self.has_sim_time = True

    def local_offset_to_lat_lon(self, north_m, east_m):
        if self.home_lat is None:
            return 0.0, 0.0
        R = 6378137.0
        d_lat = north_m / R
        d_lon = east_m / (R * math.cos(math.radians(self.home_lat)))
        lat = self.home_lat + math.degrees(d_lat)
        lon = self.home_lon + math.degrees(d_lon)
        return lat, lon

    async def telemetry_position_listener(self):
        async for pos in self.drone.telemetry.position():
            self.uav_lat = pos.latitude_deg
            self.uav_lon = pos.longitude_deg
            self.uav_alt_rel = pos.relative_altitude_m

    async def telemetry_ned_listener(self):
        async for pos_vel in self.drone.telemetry.position_velocity_ned():
            self.uav_n = pos_vel.position.north_m
            self.uav_e = pos_vel.position.east_m
            self.uav_d = pos_vel.position.down_m
            self.uav_vx = pos_vel.velocity.north_m_s
            self.uav_vy = pos_vel.velocity.east_m_s
            self.uav_vz = pos_vel.velocity.down_m_s

    async def telemetry_heading_listener(self):
        async for heading in self.drone.telemetry.heading():
            self.uav_heading = heading.heading_deg

    async def telemetry_in_air_listener(self):
        async for in_air in self.drone.telemetry.in_air():
            self.uav_in_air = in_air

    async def mission_progress_listener(self):
        async for progress in self.drone.mission.mission_progress():
            self.current_wp_idx = progress.current
            self.total_wp_count = progress.total
            # Track patrol-route progress separately so the 3-item landing
            # mission does not corrupt route-completion accounting.
            if self.state != FsmState.STATE_LANDING and progress.total == len(self.patrol_wps):
                self.patrol_wp_reached = max(getattr(self, 'patrol_wp_reached', 0), progress.current)

    async def upload_patrol_mission(self):
        mission_items = []
        for idx, wp in enumerate(self.patrol_wps):
            lat, lon = self.local_offset_to_lat_lon(wp[0], wp[1])
            mission_items.append(
                MissionItem(
                    latitude_deg=lat,
                    longitude_deg=lon,
                    relative_altitude_m=float(wp[2]),
                    speed_m_s=12.0,
                    is_fly_through=True,
                    gimbal_pitch_deg=0.0,
                    gimbal_yaw_deg=0.0,
                    camera_action=MissionItem.CameraAction.NONE,
                    loiter_time_s=0.0,
                    camera_photo_interval_s=0.0,
                    acceptance_radius_m=60.0,
                    yaw_deg=float('nan'),
                    camera_photo_distance_m=0.0,
                    vehicle_action=MissionItem.VehicleAction.NONE
                )
            )
        plan = MissionPlan(mission_items=mission_items)
        await self.drone.mission.clear_mission()
        await self.drone.mission.upload_mission(plan)
        self.get_logger().info(f"Patrol mission uploaded with {len(mission_items)} items.")

    async def offboard_sender_loop(self):
        while rclpy.ok():
            if self.offboard_active:
                try:
                    # Enforce limits: V trim ~18 m/s, roll saturated to [-45, 45], yaw rate limit is managed in loop
                    roll = np.clip(self.target_roll, -45.0, 45.0)
                    pitch = np.clip(self.target_pitch, -15.0, 15.0)
                    yaw = self.target_yaw % 360.0
                    thrust = np.clip(self.target_thrust, 0.0, 1.0)

                    await self.drone.offboard.set_attitude(
                        Attitude(
                            roll_deg=float(roll),
                            pitch_deg=float(pitch),
                            yaw_deg=float(yaw),
                            thrust_value=float(thrust)
                        )
                    )
                except Exception:
                    pass
            await asyncio.sleep(0.05)

    def get_high_priority_fov_event(self):
        for det in self.fov_detections:
            if det.event_id in self.active_events:
                evt = self.active_events[det.event_id]
                if not (evt.active and evt.priority >= self.event_priority_thresh):
                    continue
                # Skip events this UAV investigated within the cooldown window
                # (prevents PATROL<->INVESTIGATION chatter on a persistent event).
                last = self.investigated_event_times.get(evt.id)
                if last is not None and (self.t_sim - last) < self.reinvestigation_cooldown_s:
                    continue
                return evt
        return None

    def get_nearest_thermal_or_cue(self):
        nearest_dist = 999999.0
        nearest_pos = None

        for t in self.active_thermals:
            dx = t.center_north_m - self.uav_n
            dy = t.center_east_m - self.uav_e
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_pos = (t.center_north_m, t.center_east_m)

        for cue in self.thermal_cues:
            dx = cue[1] - self.uav_n
            dy = cue[2] - self.uav_e
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < nearest_dist and dist <= self.max_cue_distance:
                nearest_dist = dist
                nearest_pos = (cue[1], cue[2])

        return nearest_pos

    def get_nearest_thermal_cue(self):
        nearest_dist = 999999.0
        nearest_cue = None

        for cue in self.thermal_cues:
            dx = cue[1] - self.uav_n
            dy = cue[2] - self.uav_e
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < nearest_dist and dist <= self.max_cue_distance:
                nearest_dist = dist
                nearest_cue = cue

        return nearest_cue

    def update_glide_return_heading(self):
        if len(self.patrol_wps) == 0:
            return
        wp_idx = self.current_wp_idx
        if wp_idx >= len(self.patrol_wps):
            wp_idx = 0
            
        wp = self.patrol_wps[wp_idx]
        dx = wp[0] - self.uav_n
        dy = wp[1] - self.uav_e
        
        heading_rad = math.atan2(dy, dx)
        self.target_yaw = math.degrees(heading_rad) % 360.0

    def get_distance_to_next_patrol_wp(self):
        if len(self.patrol_wps) == 0:
            return 999999.0
        wp_idx = self.current_wp_idx
        if wp_idx >= len(self.patrol_wps):
            wp_idx = 0
            
        wp = self.patrol_wps[wp_idx]
        dx = wp[0] - self.uav_n
        dy = wp[1] - self.uav_e
        return math.sqrt(dx*dx + dy*dy)

    def publish_thermal_cue(self):
        now = self.t_sim
        if not hasattr(self, 'last_cue_publish_time'):
            self.last_cue_publish_time = 0.0
            
        if now - self.last_cue_publish_time >= 1.0:
            self.last_cue_publish_time = now
            
            cue = ThermalCue()
            sec = int(self.t_sim)
            cue.stamp.sec = sec
            cue.stamp.nanosec = int((self.t_sim - sec) * 1e9)
            cue.vehicle_id = self.vehicle_id
            cue.latitude_deg = self.uav_lat
            cue.longitude_deg = self.uav_lon
            cue.relative_altitude_m = self.uav_alt_rel
            cue.north_m = self.uav_n
            cue.east_m = self.uav_e
            cue.w_peak_mps = self.current_wind
            cue.radius_m = 100.0
            self.cue_pub.publish(cue)

    async def upload_and_start_landing(self):
        self.get_logger().info("Generating landing trajectory waypoints...")
        lat1, lon1 = self.local_offset_to_lat_lon(200.0, 0.0)
        lat2, lon2 = self.local_offset_to_lat_lon(100.0, 0.0)
        lat3, lon3 = self.home_lat, self.home_lon

        landing_items = [
            MissionItem(
                latitude_deg=lat1,
                longitude_deg=lon1,
                relative_altitude_m=35.0,
                speed_m_s=12.0,
                is_fly_through=True,
                gimbal_pitch_deg=0.0,
                gimbal_yaw_deg=0.0,
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=0.0,
                camera_photo_interval_s=0.0,
                acceptance_radius_m=10.0,
                yaw_deg=180.0,
                camera_photo_distance_m=0.0,
                vehicle_action=MissionItem.VehicleAction.NONE
            ),
            MissionItem(
                latitude_deg=lat2,
                longitude_deg=lon2,
                relative_altitude_m=15.0,
                speed_m_s=11.0,
                is_fly_through=True,
                gimbal_pitch_deg=0.0,
                gimbal_yaw_deg=0.0,
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=0.0,
                camera_photo_interval_s=0.0,
                acceptance_radius_m=10.0,
                yaw_deg=180.0,
                camera_photo_distance_m=0.0,
                vehicle_action=MissionItem.VehicleAction.NONE
            ),
            MissionItem(
                latitude_deg=lat3,
                longitude_deg=lon3,
                relative_altitude_m=0.0,
                speed_m_s=10.0,
                is_fly_through=False,
                gimbal_pitch_deg=0.0,
                gimbal_yaw_deg=0.0,
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=0.0,
                camera_photo_interval_s=0.0,
                acceptance_radius_m=10.0,
                yaw_deg=180.0,
                camera_photo_distance_m=0.0,
                vehicle_action=MissionItem.VehicleAction.LAND
            )
        ]

        try:
            plan = MissionPlan(mission_items=landing_items)
            await self.drone.mission.clear_mission()
            await self.drone.mission.upload_mission(plan)
            await asyncio.sleep(0.2)
            await self.drone.mission.start_mission()
            self.get_logger().info("Landing trajectory mission uploaded and started successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to start landing mission: {e}")

    def thermal_entry_confirmed(self, now):
        """True once lift has stayed above the (higher) entry threshold
        continuously for thermal_entry_confirm_s. Resets when lift drops."""
        if self.current_wind >= self.thermal_entry_lift:
            if self.lift_above_since is None:
                self.lift_above_since = now
            return (now - self.lift_above_since) >= self.thermal_entry_confirm_s
        self.lift_above_since = None
        return False

    def can_transition(self, new_state):
        """Enforce minimum state dwell + transition cooldown to suppress
        chatter. LANDING (battery reserve) is safety-critical and always
        allowed. Returns True if the transition may proceed; otherwise
        increments the rejected-transition counter."""
        if new_state == FsmState.STATE_LANDING:
            return True
        now = self.t_sim
        if (now - self.state_entry_time) < self.min_state_dwell_s or \
           (now - self.last_transition_time) < self.transition_cooldown_s:
            self.rejected_transitions += 1
            return False
        return True

    async def transition_to(self, new_state):
        state_names = {
            FsmState.STATE_PATROL: "PATROL",
            FsmState.STATE_EVENT_INVESTIGATION: "EVENT_INVESTIGATION",
            FsmState.STATE_THERMAL_SEARCH: "THERMAL_SEARCH",
            FsmState.STATE_THERMAL_EXPLOITATION: "THERMAL_EXPLOITATION",
            FsmState.STATE_GLIDE_RETURN: "GLIDE_RETURN",
            FsmState.STATE_LANDING: "LANDING"
        }
        self.get_logger().info(f"Transitioning FSM state: {state_names.get(self.state)} -> {state_names.get(new_state)}")

        # Count opportunistic thermal re-entries (from patrol/glide/search)
        if new_state == FsmState.STATE_THERMAL_EXPLOITATION and self.state in (
                FsmState.STATE_PATROL, FsmState.STATE_GLIDE_RETURN, FsmState.STATE_THERMAL_SEARCH):
            self.thermal_reentries += 1

        # Clean up old state
        if self.state in [FsmState.STATE_THERMAL_EXPLOITATION, FsmState.STATE_GLIDE_RETURN]:
            if self.offboard_active:
                self.offboard_active = False
                try:
                    await self.drone.offboard.stop()
                except Exception:
                    pass

        # Initialize new state
        self.state = new_state
        self.state_entry_time = self.t_sim
        self.last_transition_time = self.t_sim
        self.lift_above_since = None

        if new_state == FsmState.STATE_PATROL:
            try:
                await self.drone.mission.start_mission()
            except Exception as e:
                self.get_logger().error(f"Failed to start mission: {e}")

        elif new_state == FsmState.STATE_EVENT_INVESTIGATION:
            lat, lon = self.target_event.lat_deg, self.target_event.lon_deg
            self.investigation_start_time = self.t_sim
            try:
                abs_alt = self.home_alt + self.mission_alt
                await self.drone.action.do_orbit(
                    radius_m=50.0,
                    velocity_ms=12.0,
                    yaw_behavior=OrbitYawBehavior.HOLD_FRONT_TANGENT_TO_CIRCLE,
                    latitude_deg=lat,
                    longitude_deg=lon,
                    absolute_altitude_m=abs_alt
                )
            except Exception as e:
                self.get_logger().error(f"Failed to start orbit for event: {e}")

        elif new_state == FsmState.STATE_THERMAL_SEARCH:
            self.search_start_time = self.t_sim
            lat, lon = self.local_offset_to_lat_lon(self.search_target_n, self.search_target_e)
            try:
                abs_alt = self.home_alt + self.mission_alt
                await self.drone.action.goto_location(
                    latitude_deg=lat,
                    longitude_deg=lon,
                    absolute_altitude_m=abs_alt,
                    yaw_deg=0.0
                )
            except Exception as e:
                self.get_logger().error(f"Failed goto_location for thermal search: {e}")

        elif new_state == FsmState.STATE_THERMAL_EXPLOITATION:
            self.last_usable_lift_time = self.t_sim
            self.target_roll = 30.0
            self.target_pitch = 2.0
            self.target_yaw = self.uav_heading
            self.target_thrust = 0.0
            
            try:
                await self.drone.offboard.set_attitude(
                    Attitude(
                        roll_deg=self.target_roll,
                        pitch_deg=self.target_pitch,
                        yaw_deg=self.target_yaw,
                        thrust_value=self.target_thrust
                    )
                )
                self.offboard_active = True
                await self.drone.offboard.start()
            except Exception as e:
                self.get_logger().error(f"Failed to start offboard for exploitation: {e}")

        elif new_state == FsmState.STATE_GLIDE_RETURN:
            self.target_roll = 0.0
            self.target_pitch = 2.0
            self.update_glide_return_heading()
            self.target_thrust = 0.0
            
            try:
                await self.drone.offboard.set_attitude(
                    Attitude(
                        roll_deg=self.target_roll,
                        pitch_deg=self.target_pitch,
                        yaw_deg=self.target_yaw,
                        thrust_value=self.target_thrust
                    )
                )
                self.offboard_active = True
                await self.drone.offboard.start()
            except Exception as e:
                self.get_logger().error(f"Failed to start offboard for glide return: {e}")

        elif new_state == FsmState.STATE_LANDING:
            await self.upload_and_start_landing()

    async def fsm_tick(self):
        state_names = {
            FsmState.STATE_PATROL: "PATROL",
            FsmState.STATE_EVENT_INVESTIGATION: "EVENT_INVESTIGATION",
            FsmState.STATE_THERMAL_SEARCH: "THERMAL_SEARCH",
            FsmState.STATE_THERMAL_EXPLOITATION: "THERMAL_EXPLOITATION",
            FsmState.STATE_GLIDE_RETURN: "GLIDE_RETURN",
            FsmState.STATE_LANDING: "LANDING"
        }
        
        # 1. Publish FSM State & Telemetry ROS 2 messages
        now = self.t_sim
        if self.last_fsm_tick_time == 0.0:
            self.last_fsm_tick_time = now
            dt = 0.0
        else:
            dt = now - self.last_fsm_tick_time
        self.last_fsm_tick_time = now

        state_msg = FsmState()
        sec = int(self.t_sim)
        state_msg.stamp.sec = sec
        state_msg.stamp.nanosec = int((self.t_sim - sec) * 1e9)
        state_msg.vehicle_id = self.vehicle_id
        state_msg.state = self.state
        state_msg.state_name = state_names.get(self.state, "UNKNOWN")
        state_msg.rejected_transitions = int(self.rejected_transitions)
        state_msg.thermal_reentries = int(self.thermal_reentries)
        self.fsm_pub.publish(state_msg)

        telemetry_msg = TelemetryExchange()
        telemetry_msg.stamp.sec = sec
        telemetry_msg.stamp.nanosec = int((self.t_sim - sec) * 1e9)
        telemetry_msg.vehicle_id = self.vehicle_id
        telemetry_msg.latitude_deg = self.uav_lat
        telemetry_msg.longitude_deg = self.uav_lon
        telemetry_msg.relative_altitude_m = self.uav_alt_rel
        telemetry_msg.north_m = self.uav_n
        telemetry_msg.east_m = self.uav_e
        telemetry_msg.vx_mps = self.uav_vx
        telemetry_msg.vy_mps = self.uav_vy
        telemetry_msg.vz_mps = self.uav_vz
        telemetry_msg.current_state = self.state
        telemetry_msg.current_wp_idx = int(getattr(self, 'patrol_wp_reached', 0))
        telemetry_msg.total_wp_count = len(self.patrol_wps)
        self.telemetry_pub.publish(telemetry_msg)

        # Prune older thermal cues
        self.thermal_cues = [c for c in self.thermal_cues if now - c[0] < 60.0]

        # 2. Check global reserve trigger first (highest priority)
        if self.current_soc <= self.reserve_soc and self.state != FsmState.STATE_LANDING:
            self.get_logger().warn(f"Battery low (SoC: {self.current_soc:.2f}% <= {self.reserve_soc}%). Triggering LANDING!")
            await self.transition_to(FsmState.STATE_LANDING)
            return

        # 3. State Machine Transitions & Behaviors
        if self.state == FsmState.STATE_PATROL:
            # Check for event detection
            if self.enable_event_investigation:
                high_priority_evt = self.get_high_priority_fov_event()
                if high_priority_evt is not None and self.can_transition(FsmState.STATE_EVENT_INVESTIGATION):
                    self.target_event = high_priority_evt
                    self.get_logger().info(f"High-priority event {high_priority_evt.id} detected! Transitioning to EVENT_INVESTIGATION.")
                    await self.transition_to(FsmState.STATE_EVENT_INVESTIGATION)
                    return
 
            # Opportunistic thermal entry with hysteresis + entry confirmation:
            # lift must exceed the (higher) entry threshold continuously for
            # thermal_entry_confirm_s before committing. Altitude guards prevent
            # motor-off thermalling during takeoff and against the ceiling.
            if self.enable_soaring and 30.0 <= self.uav_alt_rel < self.thermal_ceiling_m - 20.0 \
                    and self.thermal_entry_confirmed(now):
                if self.can_transition(FsmState.STATE_THERMAL_EXPLOITATION):
                    self.get_logger().info(f"Confirmed thermal lift={self.current_wind:.2f} m/s >= {self.thermal_entry_lift} m/s. PATROL -> THERMAL_EXPLOITATION.")
                    await self.transition_to(FsmState.STATE_THERMAL_EXPLOITATION)
                    return

            # Check for thermal search trigger (low SoC). No point searching for
            # lift while already at the thermalling ceiling (prevents the
            # SEARCH->EXPLOIT->ceiling-exit loop).
            if self.enable_soaring and self.current_soc <= self.search_soc \
                    and self.uav_alt_rel < self.thermal_ceiling_m - 30.0:
                target_pos = self.get_nearest_thermal_or_cue()
                if target_pos is not None:
                    self.search_target_n, self.search_target_e = target_pos
                    self.get_logger().info(f"Low energy (SoC: {self.current_soc:.2f}%). Target search at North={self.search_target_n:.1f}, East={self.search_target_e:.1f}. Transitioning to THERMAL_SEARCH.")
                    await self.transition_to(FsmState.STATE_THERMAL_SEARCH)
                    return
 
            # Check if thermal cue received and we are meaningfully low
            # (relative to the hard search threshold so the takeoff-climb SOC
            # drop does not immediately divert the whole fleet off-route)
            if self.enable_soaring and self.current_soc < self.search_soc + 5.0 \
                    and len(self.thermal_cues) > 0 \
                    and self.uav_alt_rel < self.thermal_ceiling_m - 30.0:
                nearest_cue = self.get_nearest_thermal_cue()
                if nearest_cue is not None:
                    self.search_target_n, self.search_target_e = nearest_cue[1], nearest_cue[2]
                    self.get_logger().info(f"Received swarm thermal cue! Target search at North={self.search_target_n:.1f}, East={self.search_target_e:.1f}. Transitioning to THERMAL_SEARCH.")
                    await self.transition_to(FsmState.STATE_THERMAL_SEARCH)
                    return

        elif self.state == FsmState.STATE_EVENT_INVESTIGATION:
            duration = now - self.investigation_start_time
            event_active = self.target_event.id in self.active_events and self.active_events[self.target_event.id].active

            # Enforce a minimum loiter so investigation is not abandoned the
            # instant the event toggles inactive (stabilizes the loiter).
            if duration < self.min_investigation_s:
                pass
            elif not event_active or duration >= self.investigation_duration:
                # Stamp this event so it is not immediately re-investigated.
                if self.target_event is not None:
                    self.investigated_event_times[self.target_event.id] = now
                if not event_active:
                    self.get_logger().info(f"Target event {self.target_event.id} expired/cleared.")
                else:
                    self.get_logger().info(f"Investigation duration ({duration:.1f}s) met.")

                if self.enable_soaring and self.current_soc <= self.search_soc:
                    target_pos = self.get_nearest_thermal_or_cue()
                    if target_pos is not None:
                        self.search_target_n, self.search_target_e = target_pos
                        await self.transition_to(FsmState.STATE_THERMAL_SEARCH)
                        return

                await self.transition_to(FsmState.STATE_PATROL)
                return

        elif self.state == FsmState.STATE_THERMAL_SEARCH:
            # HP event response outranks the energy-search objective above reserve
            if self.enable_event_investigation:
                high_priority_evt = self.get_high_priority_fov_event()
                if high_priority_evt is not None:
                    self.target_event = high_priority_evt
                    self.get_logger().info(f"High-priority event {high_priority_evt.id} detected during search! Transitioning to EVENT_INVESTIGATION.")
                    await self.transition_to(FsmState.STATE_EVENT_INVESTIGATION)
                    return

            if self.thermal_entry_confirmed(now) \
                    and 30.0 <= self.uav_alt_rel < self.thermal_ceiling_m - 20.0:
                self.get_logger().info(f"Confirmed lift (w_i: {self.current_wind:.2f} m/s >= {self.thermal_entry_lift} m/s)! THERMAL_SEARCH -> THERMAL_EXPLOITATION.")
                await self.transition_to(FsmState.STATE_THERMAL_EXPLOITATION)
                return

            if now - self.search_start_time >= self.search_timeout_s:
                self.get_logger().info("Thermal search timed out without finding lift. Returning to PATROL.")
                await self.transition_to(FsmState.STATE_PATROL)
                return

        elif self.state == FsmState.STATE_THERMAL_EXPLOITATION:
            self.publish_thermal_cue()
            
            # Enforce heading rate: 0.377 rad/s (approx 21.6 deg/s) coordinated turn target
            yaw_rate_dps = 21.6
            self.target_yaw = (self.target_yaw + yaw_rate_dps * dt) % 360.0

            # On exit: with a healthy battery resume the mission under power
            # (PATROL); glide back only when energy-constrained.
            exit_state = FsmState.STATE_GLIDE_RETURN
            if self.current_soc >= self.search_soc + 15.0:
                exit_state = FsmState.STATE_PATROL

            # Exit only when lift falls below the LOWER exit threshold (hysteresis
            # band vs the higher entry threshold) for thermal_loss_timeout_s, and
            # only after the minimum dwell has elapsed.
            if self.current_wind < self.thermal_exit_lift:
                if now - self.last_usable_lift_time > self.thermal_loss_timeout_s \
                        and self.can_transition(exit_state):
                    self.get_logger().info(f"Lift below exit threshold ({self.thermal_exit_lift} m/s) for >{self.thermal_loss_timeout_s}s. Exiting to {'PATROL' if exit_state == FsmState.STATE_PATROL else 'GLIDE_RETURN'}.")
                    await self.transition_to(exit_state)
                    return
            else:
                self.last_usable_lift_time = now

            if self.uav_alt_rel >= self.thermal_ceiling_m:
                self.get_logger().info(f"Ceiling altitude reached ({self.uav_alt_rel:.1f}m >= {self.thermal_ceiling_m}m). Exiting to {'PATROL' if exit_state == FsmState.STATE_PATROL else 'GLIDE_RETURN'}.")
                await self.transition_to(exit_state)
                return

        elif self.state == FsmState.STATE_GLIDE_RETURN:
            # HP event response outranks the glide-return convenience
            if self.enable_event_investigation:
                high_priority_evt = self.get_high_priority_fov_event()
                if high_priority_evt is not None:
                    self.target_event = high_priority_evt
                    self.get_logger().info(f"High-priority event {high_priority_evt.id} detected during glide! Transitioning to EVENT_INVESTIGATION.")
                    await self.transition_to(FsmState.STATE_EVENT_INVESTIGATION)
                    return

            # Opportunistic re-exploitation: if gliding through another thermal,
            # exploit it — but require confirmed lift + dwell/cooldown so a UAV
            # cannot ping-pong between GLIDE_RETURN and THERMAL_EXPLOITATION.
            if self.enable_soaring and 30.0 <= self.uav_alt_rel < self.thermal_ceiling_m - 20.0 \
                    and self.thermal_entry_confirmed(now) \
                    and self.can_transition(FsmState.STATE_THERMAL_EXPLOITATION):
                self.get_logger().info(f"Confirmed re-exploitation during glide! Lift={self.current_wind:.2f} m/s. GLIDE_RETURN -> THERMAL_EXPLOITATION.")
                await self.transition_to(FsmState.STATE_THERMAL_EXPLOITATION)
                return

            self.update_glide_return_heading()
            # Fixed-wing aircraft turn by banking: steer the glide with a
            # proportional roll command toward the return heading (a pure yaw
            # setpoint with wings level is ignored and the glide never turns).
            heading_err = (self.target_yaw - self.uav_heading + 540.0) % 360.0 - 180.0
            self.target_roll = float(np.clip(0.8 * heading_err, -30.0, 30.0))
            dist_to_wp = self.get_distance_to_next_patrol_wp()
            if dist_to_wp < 50.0 or self.uav_alt_rel < 100.0:
                if dist_to_wp < 50.0:
                    self.get_logger().info(f"Rejoined patrol route (distance to WP: {dist_to_wp:.1f}m < 50m). Transitioning to PATROL.")
                else:
                    self.get_logger().info(f"Altitude low ({self.uav_alt_rel:.1f}m < 100m) during glide return. Transitioning to PATROL.")
                await self.transition_to(FsmState.STATE_PATROL)
                return

        elif self.state == FsmState.STATE_LANDING:
            if not self.uav_in_air:
                self.get_logger().info("Touchdown! Disarming...")
                try:
                    await self.drone.action.disarm()
                except Exception:
                    pass

    async def async_main(self):
        # MAVSDK offboard remote port — must match px4-rc.mavlink's
        # udp_offboard_port_remote (14640+instance), unique for all IDs incl. >=10.
        port = 14640 + self.vehicle_id
        self.get_logger().info(f"Connecting to MAVSDK drone on port {port}...")
        await self.drone.connect(system_address=f"udp://:{port}")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("MAVSDK Connected!")
                break

        self.get_logger().info("Waiting for EKF2 convergence...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("EKF2 Converged.")
                break

        self.get_logger().info("Getting Home Position...")
        async for home in self.drone.telemetry.home():
            self.home_lat = home.latitude_deg
            self.home_lon = home.longitude_deg
            self.home_alt = home.absolute_altitude_m
            self.get_logger().info(f"Home position: Lat={self.home_lat:.6f}, Lon={self.home_lon:.6f}, Alt={self.home_alt:.2f}")
            break

        # Raise MAVSDK telemetry stream rates so position/velocity used for
        # FSM decisions and logging stay fresh under multi-UAV load (the
        # default rates lag by seconds with 6+ vehicles).
        for setter, rate in [
                (self.drone.telemetry.set_rate_position, 10.0),
                (self.drone.telemetry.set_rate_position_velocity_ned, 10.0),
                (self.drone.telemetry.set_rate_in_air, 2.0),
                (self.drone.telemetry.set_rate_attitude_euler, 5.0)]:
            try:
                await setter(rate)
            except Exception as e:
                self.get_logger().warn(f"Telemetry rate setup failed: {e}")

        # Start telemetry listeners
        asyncio.create_task(self.telemetry_position_listener())
        asyncio.create_task(self.telemetry_ned_listener())
        asyncio.create_task(self.telemetry_heading_listener())
        asyncio.create_task(self.telemetry_in_air_listener())
        asyncio.create_task(self.mission_progress_listener())
        asyncio.create_task(self.offboard_sender_loop())

        # Upload patrol mission
        await self.upload_patrol_mission()

        # Arm & Start patrol
        self.get_logger().info("Arming vehicle and starting PATROL mission...")
        
        # Wait for estimators to settle
        await asyncio.sleep(3.0)
        
        armed = False
        for attempt in range(30):
            try:
                await self.drone.action.arm()
                self.get_logger().info("Arming successful!")
                armed = True
                break
            except Exception as e:
                self.get_logger().warn(f"Arming attempt {attempt+1} failed: {e}. Retrying in 2s...")
                await asyncio.sleep(2.0)
                
        if armed:
            for attempt in range(5):
                try:
                    await self.drone.mission.start_mission()
                    self.get_logger().info("Mission start successful!")
                    break
                except Exception as e:
                    self.get_logger().warn(f"Start mission attempt {attempt+1} failed: {e}. Retrying in 2s...")
                    await asyncio.sleep(2.0)
            
        self.state = FsmState.STATE_PATROL

        # Main loop
        dt = 1.0 / self.update_rate_hz
        while rclpy.ok():
            try:
                if self.has_sim_time:
                    await self.fsm_tick()
            except Exception as e:
                self.get_logger().error(f"Error in FSM tick: {e}")
            await asyncio.sleep(dt)

def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()

    # Spin in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run async main loop
    try:
        asyncio.run(node.async_main())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
