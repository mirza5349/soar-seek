#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from soarer_msgs.msg import GroundEvent, GroundEventArray
from px4_msgs.msg import VehicleLocalPosition

class GroundEventNode(Node):
    """Temporally activated stochastic ground events.

    Events arrive as a seeded homogeneous Poisson process at `event_rate_hz`,
    stay active for `event_lifetime_s`, and carry a priority in 1..5.
    `high_priority_ratio` controls the probability of priority >= 3.
    An initial batch of `initial_event_count` events is spawned at mission
    start so short-horizon runs still encounter active events.
    """

    def __init__(self):
        super().__init__('ground_event_node')

        # Declare parameters
        self.declare_parameter('seed', 123)
        self.declare_parameter('event_rate_hz', 0.05)        # Poisson arrival rate (events/s)
        self.declare_parameter('high_priority_ratio', 0.4)   # P(priority >= 3)
        self.declare_parameter('area_size_m', 2000.0)
        self.declare_parameter('event_lifetime_s', 90.0)     # active duration per event
        self.declare_parameter('event_radius_m', 20.0)       # physical footprint radius
        self.declare_parameter('max_active_events', 30)
        self.declare_parameter('initial_event_count', 12)
        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('ref_lat_deg', 47.3769)
        self.declare_parameter('ref_lon_deg', 8.5417)

        # Get parameters
        self.seed = self.get_parameter('seed').value
        self.event_rate = self.get_parameter('event_rate_hz').value
        self.hp_ratio = self.get_parameter('high_priority_ratio').value
        self.area_size_m = self.get_parameter('area_size_m').value
        self.event_lifetime = self.get_parameter('event_lifetime_s').value
        self.event_radius = self.get_parameter('event_radius_m').value
        self.max_active_events = self.get_parameter('max_active_events').value
        self.initial_event_count = self.get_parameter('initial_event_count').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.ref_lat = self.get_parameter('ref_lat_deg').value
        self.ref_lon = self.get_parameter('ref_lon_deg').value

        # Seed the RNG
        self.rng = np.random.default_rng(self.seed)

        # Event tracking
        self.events = []
        self.next_event_id = 1
        self.t_sim = 0.0
        self.next_arrival_time = None
        self.spawned_initial = False

        # Subscriptions
        from rclpy.qos import qos_profile_sensor_data
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile_sensor_data
        )

        # Publishers
        self.events_pub = self.create_publisher(GroundEventArray, '/soarer/events', 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.timer_callback)
        self.get_logger().info(
            f"Ground Event Node started: seed={self.seed}, rate={self.event_rate} Hz, "
            f"hp_ratio={self.hp_ratio}, lifetime={self.event_lifetime}s, radius={self.event_radius}m."
        )

    def position_callback(self, msg):
        t = msg.timestamp / 1e6
        if t > self.t_sim:
            self.t_sim = t

    def draw_priority(self):
        # priority >= 3 with probability hp_ratio; uniform within {3,4,5} and {1,2}
        if self.rng.random() < self.hp_ratio:
            return int(self.rng.choice([3, 4, 5]))
        return int(self.rng.choice([1, 2]))

    def spawn_event(self, spawn_time):
        # Draw location in actual flight region
        north = float(self.rng.uniform(100.0, 2800.0))
        east = float(self.rng.uniform(-2400.0, 2900.0))

        # Coordinate conversion to GPS (flat-earth model)
        lat_deg = self.ref_lat + (north / 111132.954)
        lon_deg = self.ref_lon + (east / (111412.84 * np.cos(np.radians(self.ref_lat))))

        evt = {
            'id': self.next_event_id,
            'north_m': north,
            'east_m': east,
            'lat_deg': lat_deg,
            'lon_deg': lon_deg,
            'event_type': int(self.rng.choice([GroundEvent.SEARCH_TARGET, GroundEvent.RESCUE])),
            'priority': self.draw_priority(),
            'spawn_time': spawn_time,
            'expiry_time': spawn_time + self.event_lifetime,
            'radius_m': self.event_radius,
            'active': True
        }
        self.events.append(evt)
        self.next_event_id += 1
        return evt

    def timer_callback(self):
        if self.t_sim <= 0.0:
            return  # wait for simulation clock

        # One-time initial batch so the surveillance area is populated at start
        if not self.spawned_initial:
            for _ in range(int(self.initial_event_count)):
                self.spawn_event(self.t_sim)
            # First Poisson arrival after the initial batch
            if self.event_rate > 0.0:
                self.next_arrival_time = self.t_sim + float(self.rng.exponential(1.0 / self.event_rate))
            self.spawned_initial = True
            self.get_logger().info(f"Spawned initial batch of {self.initial_event_count} events at t={self.t_sim:.1f}s.")

        # Seeded Poisson arrivals
        while (self.next_arrival_time is not None
               and self.t_sim >= self.next_arrival_time):
            n_active = sum(1 for e in self.events if e['active'])
            if n_active < self.max_active_events:
                self.spawn_event(self.next_arrival_time)
            self.next_arrival_time += float(self.rng.exponential(1.0 / self.event_rate))

        # Update event active states based on self.t_sim
        for evt in self.events:
            evt['active'] = (evt['spawn_time'] <= self.t_sim < evt['expiry_time'])

        # Keep the list size bounded by pruning events that expired more than 10 mins ago
        self.events = [e for e in self.events if e['active'] or (self.t_sim - e['expiry_time']) < 600.0]

        # Publish events array
        array_msg = GroundEventArray()
        array_msg.stamp = self.get_clock().now().to_msg()
        array_msg.sim_time_s = self.t_sim

        for evt in self.events:
            msg = GroundEvent()
            msg.id = evt['id']
            msg.north_m = evt['north_m']
            msg.east_m = evt['east_m']
            msg.lat_deg = evt['lat_deg']
            msg.lon_deg = evt['lon_deg']
            msg.event_type = evt['event_type']
            msg.priority = evt['priority']
            msg.spawn_time = evt['spawn_time']
            msg.expiry_time = evt['expiry_time']
            msg.radius_m = evt['radius_m']
            msg.active = evt['active']
            array_msg.events.append(msg)

        self.events_pub.publish(array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GroundEventNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
