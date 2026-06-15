#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from soarer_msgs.msg import BatteryEstimate, FsmState
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleStatus, VehicleThrustSetpoint, ActuatorMotors, AirspeedValidated, VehicleControlMode

class BatteryEstimatorNode(Node):
    def __init__(self):
        super().__init__('battery_estimator_node')

        # Declare parameters
        self.declare_parameter('vehicle_id', 1)
        self.declare_parameter('mass_kg', 1.50)
        self.declare_parameter('wing_area_m2', 0.90)
        self.declare_parameter('CD0', 0.012)
        self.declare_parameter('oswald_e', 0.95)
        self.declare_parameter('AR', 14.0)
        self.declare_parameter('eta_prop', 0.85)
        self.declare_parameter('rho_kg_m3', 1.225)
        self.declare_parameter('battery_capacity_wh', 76.96) # 14.8V * 5.2Ah
        self.declare_parameter('V_nom', 14.8)
        self.declare_parameter('update_rate_hz', 2.0)
        # 'aero'     : propulsion power from coupled aerodynamic state (default)
        # 'constant' : simplified constant-power discharge whenever armed
        self.declare_parameter('battery_model', 'aero')
        self.declare_parameter('constant_power_w', 55.0)

        # Get parameter values
        self.vehicle_id = self.get_parameter('vehicle_id').value
        self.mass = self.get_parameter('mass_kg').value
        self.S_w = self.get_parameter('wing_area_m2').value
        self.CD0 = self.get_parameter('CD0').value
        self.e = self.get_parameter('oswald_e').value
        self.AR = self.get_parameter('AR').value
        self.eta_prop = self.get_parameter('eta_prop').value
        self.rho = self.get_parameter('rho_kg_m3').value
        self.E_batt = self.get_parameter('battery_capacity_wh').value
        self.V_nom = self.get_parameter('V_nom').value
        self.update_rate_hz = self.get_parameter('update_rate_hz').value
        self.battery_model = self.get_parameter('battery_model').value
        self.constant_power_w = self.get_parameter('constant_power_w').value

        # Drag constant k = 1 / (pi * e * AR)
        self.k = 1.0 / (np.pi * self.e * self.AR)
        # Weight of gravity W = m * g
        self.W = self.mass * 9.80665

        # State tracking
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.has_gps_vel = False
        
        self.true_airspeed = 0.0
        self.has_airspeed = False
        
        self.roll = 0.0
        self.armed = False
        self.throttle_cmd = 0.0
        self.offboard_enabled = False
        
        # Integration tracking
        self.energy_consumed_wh = 0.0
        self.last_update_time = 0.0
        self.reported_reserve = False

        # Subscriptions
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        
        qos_profile_transient_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

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
        self.status_sub = self.create_subscription(
            VehicleStatus,
            f'/px4_{self.vehicle_id}/fmu/out/vehicle_status',
            self.status_callback,
            qos_profile_transient_best_effort
        )
        self.thrust_sub = self.create_subscription(
            VehicleThrustSetpoint,
            f'/px4_{self.vehicle_id}/fmu/out/vehicle_thrust_setpoint',
            self.thrust_callback,
            qos_profile_sensor_data
        )
        self.motors_sub = self.create_subscription(
            ActuatorMotors,
            f'/px4_{self.vehicle_id}/fmu/out/actuator_motors',
            self.motors_callback,
            qos_profile_sensor_data
        )
        self.airspeed_sub = self.create_subscription(
            AirspeedValidated,
            f'/px4_{self.vehicle_id}/fmu/out/airspeed_validated',
            self.airspeed_callback,
            qos_profile_sensor_data
        )
        self.control_mode_sub = self.create_subscription(
            VehicleControlMode,
            f'/px4_{self.vehicle_id}/fmu/out/vehicle_control_mode',
            self.control_mode_callback,
            qos_profile_transient_best_effort
        )

        self.fsm_state = None
        self.fsm_sub = self.create_subscription(
            FsmState,
            f'/soarer/fsm/px4_{self.vehicle_id}',
            self.fsm_callback,
            10
        )

        # Publisher
        self.battery_pub = self.create_publisher(
            BatteryEstimate,
            f'/soarer/battery/px4_{self.vehicle_id}',
            10
        )

        self.last_publish_time = 0.0
        self.get_logger().info(f"Battery Estimator Node started for UAV {self.vehicle_id}.")

    def position_callback(self, msg):
        self.vx = msg.vx
        self.vy = msg.vy
        self.vz = msg.vz
        self.has_gps_vel = True
        self.update_battery(msg.timestamp / 1e6)

    def attitude_callback(self, msg):
        # Convert quaternion msg.q [w, x, y, z] to roll angle phi
        w, x, y, z = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        self.roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

    def status_callback(self, msg):
        # arming_state: 2 is ARMED
        self.armed = (msg.arming_state == 2)

    def thrust_callback(self, msg):
        # For fixed-wing, setpoint along X is the throttle command
        self.throttle_cmd = msg.xyz[0]

    def motors_callback(self, msg):
        # Fallback throttle estimation from first motor command if not NaN
        val = msg.control[0]
        if not np.isnan(val):
            self.throttle_cmd = val

    def airspeed_callback(self, msg):
        self.true_airspeed = msg.true_airspeed_m_s
        self.has_airspeed = True

    def control_mode_callback(self, msg):
        self.offboard_enabled = msg.flag_control_offboard_enabled

    def fsm_callback(self, msg):
        self.fsm_state = msg.state

    def update_battery(self, current_time):
        if self.last_update_time == 0.0:
            self.last_update_time = current_time
            self.last_publish_time = current_time
            return
        
        dt = current_time - self.last_update_time
        if dt <= 0.0:
            # Out-of-order sample: keep the monotonic clock.
            return
        if dt > 1.0:
            self.last_update_time = current_time
            self.last_publish_time = current_time
            return

        self.last_update_time = current_time

        # Calculate current airspeed V (prefer true airspeed, fallback to GPS speed, fallback to trim 12 m/s)
        V = 12.0
        if self.has_airspeed and self.true_airspeed > 1.0:
            V = self.true_airspeed
        elif self.has_gps_vel:
            V = np.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
            if V < 1.0:
                V = 12.0

        # Determine motor mode based on FSM state if available
        if self.fsm_state is not None:
            # Exploitation and Glide return have motor off (0 W draw)
            motor_active = self.armed and (self.fsm_state not in [FsmState.STATE_THERMAL_EXPLOITATION, FsmState.STATE_GLIDE_RETURN])
        else:
            # Fallback to offboard check if FSM state not yet received
            motor_active = self.armed and not self.offboard_enabled

        P_total = 0.0
        mode_str = "motor-off (thermalling/glide)"

        if self.battery_model == 'constant':
            # Simplified battery model: fixed power draw whenever armed,
            # ignoring flight state, bank angle, and motor-off soaring.
            if self.armed:
                P_total = self.constant_power_w
                mode_str = "constant power"
        elif motor_active:
            # Lift matches weight + centripetal force in coordinated turn: L = W / cos(roll)
            cos_roll = np.cos(self.roll)
            L = self.W / max(0.1, cos_roll)
            
            # Dynamic pressure q = 0.5 * rho * V^2
            q = 0.5 * self.rho * V**2
            q = max(0.1, q)

            # Lift coefficient CL = L / (q * S_w)
            CL = L / (q * self.S_w)

            # Total drag coefficient CD = CD0 + k * CL^2
            CD = self.CD0 + self.k * CL**2

            # Drag force D = CD * q * S_w
            D = CD * q * self.S_w

            # Aerodynamic power required to overcome drag: P_aero = D * V
            P_aero = D * V

            # Shaft power: P_shaft = P_aero / eta_prop
            P_shaft = P_aero / self.eta_prop

            # Total power draw
            P_total = P_shaft

            if np.abs(self.roll) >= np.radians(5.0):
                mode_str = "turning power"
            else:
                mode_str = "cruise power"

        # Integrate energy
        dE = P_total * (dt / 3600.0) # Wh
        self.energy_consumed_wh += dE

        # Remaining capacity
        energy_remaining = max(0.0, self.E_batt - self.energy_consumed_wh)
        soc_pct = (energy_remaining / self.E_batt) * 100.0

        # Current & Voltage
        voltage = self.V_nom
        current = P_total / voltage

        # Remaining time estimate
        if P_total > 0.1:
            est_remaining = (energy_remaining / P_total) * 3600.0
        else:
            est_remaining = 999999.0

        # Check reserve threshold (7% of E_batt = 0.07 * E_batt)
        reserve_threshold = 0.07 * self.E_batt
        if energy_remaining <= reserve_threshold and not self.reported_reserve:
            self.get_logger().warn(
                f"[UAV {self.vehicle_id}] BATTERY RESERVE WARNING! "
                f"Remaining energy {energy_remaining:.2f} Wh is below 7% reserve ({reserve_threshold:.2f} Wh)."
            )
            self.reported_reserve = True

        # Throttle publishing to 2 Hz simulation time (0.5s intervals)
        if current_time - self.last_publish_time >= 0.5:
            self.last_publish_time = current_time

            # Publish battery estimate
            msg = BatteryEstimate()
            sec = int(current_time)
            msg.stamp.sec = sec
            msg.stamp.nanosec = int((current_time - sec) * 1e9)
            msg.vehicle_id = self.vehicle_id
            msg.soc_pct = float(soc_pct)
            msg.energy_consumed_wh = float(self.energy_consumed_wh)
            msg.energy_total_wh = float(self.E_batt)
            msg.power_draw_w = float(P_total)
            msg.voltage_v = float(voltage)
            msg.current_a = float(current)
            msg.est_remaining_s = float(est_remaining)
            
            self.battery_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryEstimatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
