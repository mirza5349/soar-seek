#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import socket
import struct
from soarer_msgs.msg import ThermalField, ThermalState, VerticalWind
from px4_msgs.msg import VehicleLocalPosition

class ThermalFieldNode(Node):
    def __init__(self):
        super().__init__('thermal_field_node')
        
        # Declare parameters
        self.declare_parameter('seed', 42)
        self.declare_parameter('num_thermals', 8)
        self.declare_parameter('area_size_m', 2000.0)
        self.declare_parameter('update_rate_hz', 5.0)
        self.declare_parameter('w_peak_min_mps', 1.5)
        self.declare_parameter('w_peak_max_mps', 5.0)
        self.declare_parameter('radius_min_m', 50.0)
        self.declare_parameter('radius_max_m', 200.0)
        self.declare_parameter('alt_floor_m', 50.0)
        self.declare_parameter('alt_ceiling_m', 800.0)
        self.declare_parameter('drift_speed_mps', 0.5)
        self.declare_parameter('thermal_lifetime_s', 600.0)
        self.declare_parameter('num_vehicles', 2)

        # Get parameter values
        self.seed = self.get_parameter('seed').value
        self.num_thermals = self.get_parameter('num_thermals').value
        self.area_size_m = self.get_parameter('area_size_m').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.w_peak_min = self.get_parameter('w_peak_min_mps').value
        self.w_peak_max = self.get_parameter('w_peak_max_mps').value
        self.radius_min = self.get_parameter('radius_min_m').value
        self.radius_max = self.get_parameter('radius_max_m').value
        self.alt_floor = self.get_parameter('alt_floor_m').value
        self.alt_ceiling = self.get_parameter('alt_ceiling_m').value
        self.drift_speed = self.get_parameter('drift_speed_mps').value
        self.thermal_lifetime = self.get_parameter('thermal_lifetime_s').value
        self.num_vehicles = self.get_parameter('num_vehicles').value

        # Seed the RNG
        self.rng = np.random.default_rng(self.seed)

        # Initialize thermals by packing the maximum number possible in the region [100, 2800] North and [-2400, 2900] East
        # with boundary-to-boundary separation >= 2 * radius
        self.thermals = []
        thermal_id = 1
        failed_consecutively = 0
        while failed_consecutively < 500 and len(self.thermals) < 60:
            radius = float(self.rng.uniform(self.radius_min, self.radius_max))
            initial_north = float(self.rng.uniform(100.0, 2800.0))
            initial_east = float(self.rng.uniform(-2400.0, 2900.0))
            
            if self.is_valid_position(initial_north, initial_east, radius):
                drift_angle = float(self.rng.uniform(0, 2.0 * np.pi))
                lifetime = float(self.rng.exponential(self.thermal_lifetime))
                
                self.thermals.append({
                    'id': thermal_id,
                    'initial_north': initial_north,
                    'initial_east': initial_east,
                    'center_north': initial_north,
                    'center_east': initial_east,
                    'radius': radius,
                    'core_radius': 0.36 * radius,
                    'w_peak': float(self.rng.uniform(self.w_peak_min, self.w_peak_max)),
                    'alt_floor': float(self.alt_floor),
                    'alt_ceiling': float(self.alt_ceiling),
                    'drift_angle': drift_angle,
                    'spawn_time': 0.0,
                    'expiry_time': lifetime,
                    'active': True
                })
                thermal_id += 1
                failed_consecutively = 0
            else:
                failed_consecutively += 1

        # Vehicle state tracking
        self.uav_positions = {}  # vehicle_id -> [north, east, alt_agl]
        self.uav_timestamps = {} # vehicle_id -> timestamp (s)
        self.t_sim = 0.0

        # Subscriptions
        self.subs = []
        self.wind_pubs = {}
        from rclpy.qos import qos_profile_sensor_data
        for i in range(1, self.num_vehicles + 1):
            topic = f'/px4_{i}/fmu/out/vehicle_local_position'
            # Using local variable binding for callbacks
            self.subs.append(self.create_subscription(
                VehicleLocalPosition,
                topic,
                lambda msg, uav_id=i: self.position_callback(msg, uav_id),
                qos_profile_sensor_data
            ))
            self.wind_pubs[i] = self.create_publisher(
                VerticalWind,
                f'/soarer/wind/px4_{i}',
                10
            )

        # Publisher for the whole thermal field
        self.field_pub = self.create_publisher(ThermalField, '/soarer/thermals', 10)

        # UDP socket for wind injection
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.timer_callback)
        self.get_logger().info(f"Thermal Field Node started with seed {self.seed}, {len(self.thermals)} thermals packed, {self.num_vehicles} vehicles.")

    def is_valid_position(self, north, east, radius, current_id=None):
        for th in self.thermals:
            if th['id'] == current_id or not th['active']:
                continue
            dx = north - th['center_north']
            dy = east - th['center_east']
            dist = np.sqrt(dx*dx + dy*dy)
            min_dist = radius + th['radius'] + 2.0 * max(radius, th['radius'])
            if dist < min_dist:
                return False
        return True

    def generate_thermal(self, thermal_id, current_time):
        radius = float(self.rng.uniform(self.radius_min, self.radius_max))
        initial_north, initial_east = 0.0, 0.0
        
        # Try to place the respawned thermal in a valid location
        for _ in range(500):
            initial_north = float(self.rng.uniform(100.0, 2800.0))
            initial_east = float(self.rng.uniform(-2400.0, 2900.0))
            if self.is_valid_position(initial_north, initial_east, radius, current_id=thermal_id):
                break
        else:
            initial_north = float(self.rng.uniform(100.0, 2800.0))
            initial_east = float(self.rng.uniform(-2400.0, 2900.0))

        drift_angle = float(self.rng.uniform(0, 2.0 * np.pi))
        lifetime = float(self.rng.exponential(self.thermal_lifetime))

        return {
            'id': thermal_id,
            'initial_north': initial_north,
            'initial_east': initial_east,
            'center_north': initial_north,
            'center_east': initial_east,
            'radius': radius,
            'core_radius': 0.36 * radius,
            'w_peak': float(self.rng.uniform(self.w_peak_min, self.w_peak_max)),
            'alt_floor': float(self.alt_floor),
            'alt_ceiling': float(self.alt_ceiling),
            'drift_angle': drift_angle,
            'spawn_time': current_time,
            'expiry_time': current_time + lifetime,
            'active': True
        }

    def position_callback(self, msg, uav_id):
        # Convert timestamp to seconds since boot
        t = msg.timestamp / 1e6
        self.uav_timestamps[uav_id] = t
        # Local position: x=north, y=east, z=down
        # z is NED downward, so alt AGL is -z if ref_alt/ground altitude is corrected
        # PX4 local position z is relative to takeoff point
        alt_agl = -msg.z
        self.uav_positions[uav_id] = [msg.x, msg.y, alt_agl]
        
        # Update simulation time to the latest message time
        if t > self.t_sim:
            self.t_sim = t

    def timer_callback(self):
        # Update thermal positions & active states
        for i, therm in enumerate(self.thermals):
            if not therm['active']:
                continue
            
            # If expired, respawn it
            if self.t_sim > therm['expiry_time']:
                self.thermals[i] = self.generate_thermal(therm['id'], self.t_sim)
                continue

            # Update coordinates based on constant drift speed
            elapsed = self.t_sim - therm['spawn_time']
            therm['center_north'] = therm['initial_north'] + self.drift_speed * np.cos(therm['drift_angle']) * elapsed
            therm['center_east'] = therm['initial_east'] + self.drift_speed * np.sin(therm['drift_angle']) * elapsed

        # Publish thermal field state
        field_msg = ThermalField()
        field_msg.stamp = self.get_clock().now().to_msg()
        field_msg.seed = self.seed
        field_msg.sim_time_s = self.t_sim
        for therm in self.thermals:
            state = ThermalState()
            state.id = therm['id']
            state.center_north_m = therm['center_north']
            state.center_east_m = therm['center_east']
            state.radius_m = therm['radius']
            state.core_radius_m = therm['core_radius']
            state.w_peak_mps = therm['w_peak']
            state.alt_floor_m = therm['alt_floor']
            state.alt_ceiling_m = therm['alt_ceiling']
            state.active = therm['active']
            field_msg.thermals.append(state)
        self.field_pub.publish(field_msg)

        # For each vehicle, compute wind & publish / inject
        for uav_id in range(1, self.num_vehicles + 1):
            w_total = 0.0
            contributing = []
            
            if uav_id in self.uav_positions:
                pos = self.uav_positions[uav_id]
                north, east, alt = pos[0], pos[1], pos[2]
                
                for therm in self.thermals:
                    if not therm['active']:
                        continue
                    # Check horizontal distance
                    dx = north - therm['center_north']
                    dy = east - therm['center_east']
                    dist = np.sqrt(dx*dx + dy*dy)
                    
                    if dist <= therm['radius'] and therm['alt_floor'] <= alt <= therm['alt_ceiling']:
                        # Radial profile (Allen toroidal-ring)
                        r1 = therm['core_radius']
                        w_rad = therm['w_peak'] * (dist / r1) * np.exp(0.5 * (1.0 - (dist / r1)**2))
                        # Altitude shaping
                        w_alt = np.sin(np.pi * (alt - therm['alt_floor']) / (therm['alt_ceiling'] - therm['alt_floor']))**2
                        w_total += w_rad * w_alt
                        contributing.append(therm['id'])

            # Publish VerticalWind
            wind_msg = VerticalWind()
            wind_msg.stamp = self.get_clock().now().to_msg()
            wind_msg.vehicle_id = uav_id
            if uav_id in self.uav_positions:
                wind_msg.north_m = self.uav_positions[uav_id][0]
                wind_msg.east_m = self.uav_positions[uav_id][1]
                wind_msg.alt_m = self.uav_positions[uav_id][2]
            wind_msg.w_total_mps = float(w_total)
            wind_msg.contributing_thermal_ids = contributing
            self.wind_pubs[uav_id].publish(wind_msg)

            # Direct wind injection via UDP side-channel to jsbsim_bridge
            try:
                # Send w_total as float (4 bytes) to 127.0.0.1:15000+uav_id
                data = struct.pack('f', w_total)
                self.udp_sock.sendto(data, ('127.0.0.1', 15000 + uav_id))
            except Exception as e:
                self.get_logger().error(f"Failed to inject wind to UAV {uav_id}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ThermalFieldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
