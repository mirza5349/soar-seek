#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from soarer_msgs.msg import GroundEventArray, FovDetection, FovDetectionArray
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

class FovDetectionNode(Node):
    def __init__(self):
        super().__init__('fov_detection_node')

        # Declare parameters
        self.declare_parameter('vehicle_id', 1)
        self.declare_parameter('fov_half_angle_deg', 30.0)
        self.declare_parameter('max_detection_range_m', 500.0)
        self.declare_parameter('update_rate_hz', 2.0)

        # Get parameter values
        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.fov_half_angle_rad = np.radians(self.get_parameter('fov_half_angle_deg').value)
        self.max_detection_range = self.get_parameter('max_detection_range_m').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value

        # State tracking
        self.pos = None        # [north, east, alt_agl]
        self.pos_t = 0.0       # sim time of the position sample in use
        self.q = [1.0, 0.0, 0.0, 0.0] # [w, x, y, z] attitude quaternion
        self.events = []       # list of active events

        # Subscriptions
        from rclpy.qos import qos_profile_sensor_data
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            f'/px4_{self.vehicle_id}/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile_sensor_data
        )
        self.att_sub = self.create_subscription(
            VehicleAttitude,
            f'/px4_{self.vehicle_id}/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile_sensor_data
        )
        self.events_sub = self.create_subscription(
            GroundEventArray,
            '/soarer/events',
            self.events_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            FovDetectionArray,
            f'/soarer/fov/px4_{self.vehicle_id}',
            10
        )

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.timer_callback)
        self.get_logger().info(f"FOV Detection Node started for UAV {self.vehicle_id}.")

    def position_callback(self, msg):
        # z is NED downward, so alt AGL is -z
        alt_agl = -msg.z
        self.pos = [msg.x, msg.y, alt_agl]
        self.pos_t = msg.timestamp / 1e6

    def attitude_callback(self, msg):
        # msg.q is float32[4] representing quaternion FRD -> NED
        self.q = [float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])]

    def events_callback(self, msg):
        self.events = msg.events

    def rotate_vector(self, q, v):
        # Rotate vector v by quaternion q = [w, x, y, z]
        w = q[0]
        u = np.array([q[1], q[2], q[3]])
        v_arr = np.array(v)
        return 2.0 * np.dot(u, v_arr) * u + (w*w - np.dot(u, u)) * v_arr + 2.0 * w * np.cross(u, v_arr)

    def timer_callback(self):
        if self.pos is None:
            return

        # Prepare detection array msg
        array_msg = FovDetectionArray()
        array_msg.stamp = self.get_clock().now().to_msg()
        array_msg.sim_time_s = float(self.pos_t)
        array_msg.vehicle_id = self.vehicle_id
        array_msg.fov_half_angle_rad = self.fov_half_angle_rad

        # Compute camera vector in NED frame
        # Standard camera points straight down in body frame: [0, 0, 1]
        v_cam_b = [0.0, 0.0, 1.0]
        v_cam_ned = self.rotate_vector(self.q, v_cam_b)
        v_cam_ned = v_cam_ned / np.linalg.norm(v_cam_ned)

        # Check each event
        for evt in self.events:
            if not evt.active:
                continue

            # Vector from UAV to event (NED)
            # UAV alt is altitude AGL (positive). Ground is assumed to be at z = 0.
            # So the ground event is at z = 0, meaning relative z is: ground_z - uav_z = 0 - (-alt_agl) = alt_agl
            # Relative vector (from UAV to event):
            dx = evt.north_m - self.pos[0]
            dy = evt.east_m - self.pos[1]
            dz = self.pos[2] # ground is dz meters below the UAV
            
            v_rel = np.array([dx, dy, dz])
            slant_range = np.linalg.norm(v_rel)

            if slant_range > self.max_detection_range or slant_range < 0.1:
                continue

            # Normalized relative vector
            v_rel_norm = v_rel / slant_range

            # Calculate angle between camera axis and event vector
            cos_angle = np.dot(v_cam_ned, v_rel_norm)
            # Clamp for safety
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = np.arccos(cos_angle)

            # Finite event footprint widens the effective detection cone:
            # the event is seen if any part of its radius_m disc enters the FOV.
            effective_half_angle = self.fov_half_angle_rad + np.arctan2(max(0.0, evt.radius_m), slant_range)

            if angle <= effective_half_angle:
                # Event is inside camera cone!
                det = FovDetection()
                det.event_id = evt.id
                det.range_m = float(slant_range)
                
                # Compute relative bearing and elevation in body/heading-relative frame if desired,
                # or simply relative to vehicle yaw. Let's compute relative bearing in NED for simplicity.
                # bearing: angle in horizontal plane relative to north (or yaw if we project it).
                # The user request asks for bearing_rad, elevation_rad.
                # Let's compute horizontal bearing in NED:
                bearing = np.arctan2(dy, dx)
                # elevation: angle below horizontal plane (so 90 deg = straight down)
                # elevation_rad = arcsin(dz / slant_range)
                elevation = np.arcsin(dz / slant_range)

                det.bearing_rad = float(bearing)
                det.elevation_rad = float(elevation)
                
                # Confidence degrades with distance
                confidence = 1.0 - (slant_range / self.max_detection_range)**2
                det.confidence = float(np.clip(confidence, 0.0, 1.0))
                
                array_msg.detections.append(det)

        self.detection_pub.publish(array_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FovDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
