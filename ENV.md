# Environmental & Sensing/Energy Layer (Step 4a)

This document describes the design, configuration, and verification of the environmental simulation layer (thermals and ground events) and the per-UAV sensing/estimation layer (FOV camera detection and battery state-of-charge estimator).

---

## 1. Master Seed & Parameter Configs

To ensure complete repeatability and determinism across flight trials, a single fixed master seed controls the random processes.

### Master Seed Configs
- **Thermal Field Generator**: `seed = 42`
- **Ground Event Poisson Process**: `seed = 123`

### Node Parameters
The nodes read default parameters from [config.yaml](file:///home/px4_sitl/sim_paper/ros2_ws/src/soarer_env/config/config.yaml):
- **Thermal Drift Speed**: `0.5 m/s`
- **Thermal Lifespan**: `600 s` (mean of exponential distribution)
- **Ground Event Rate**: `0.05 Hz` (λ = 1 event every 20 seconds on average)
- **Ground Event Lifespan**: `300 s` (mean of exponential distribution)
- **Camera FOV**: `30.0 degrees` half-cone angle (60.0 degrees full angle)
- **Max Camera Range**: `500.0 m`

---

## 2. ROS 2 Topic Map (ROS_DOMAIN_ID = 10)

The system runs on the dedicated domain `ROS_DOMAIN_ID = 10` for complete isolation from default nodes.

### Global Environmental Topics

| Topic Name | Message Type | Publisher Node | Description |
|---|---|---|---|
| `/soarer/thermals` | `soarer_msgs/msg/ThermalField` | `thermal_field_node` | State array of all 8 time-varying thermals |
| `/soarer/events` | `soarer_msgs/msg/GroundEventArray` | `ground_event_node` | Active ground-level target / rescue events |

### Per-UAV Namespaced Topics (for UAV $i$)

| Topic Name | Message Type | Pub / Sub | Description |
|---|---|---|---|
| `/px4_{i}/fmu/out/vehicle_local_position` | `px4_msgs/msg/VehicleLocalPosition` | Sub | Position of UAV $i$ in NED frame |
| `/px4_{i}/fmu/out/vehicle_attitude` | `px4_msgs/msg/VehicleAttitude` | Sub | Attitude quaternion of UAV $i$ |
| `/px4_{i}/fmu/out/airspeed_validated` | `px4_msgs/msg/AirspeedValidated` | Sub | True airspeed of UAV $i$ |
| `/soarer/wind/px4_{i}` | `soarer_msgs/msg/VerticalWind` | Pub | Wind computed at UAV $i$'s coordinate |
| `/soarer/fov/px4_{i}` | `soarer_msgs/msg/FovDetectionArray` | Pub | Array of ground events visible in UAV $i$'s camera |
| `/soarer/battery/px4_{i}` | `soarer_msgs/msg/BatteryEstimate` | Pub | Energy state-of-charge estimator for UAV $i$ |

---

## 3. Wind-Injection Path (Direct UDP Side-Channel)

A C++ patch was implemented in the JSBSim Bridge to support real-time external wind injection.

### Bridge Patch
- **Socket Receiver**: Inside [jsbsim_bridge.cpp](file:///home/px4_sitl/sim_paper/PX4-Autopilot/Tools/simulation/jsbsim/jsbsim_bridge/src/jsbsim_bridge.cpp), a non-blocking UDP socket is bound to localhost on port `15000 + PX4_INSTANCE` (e.g. `15001` for UAV 1).
- **Execution Loop**: In the `Run()` loop, the bridge polls this socket for a 4-byte `float` (representing vertical wind in m/s, positive = updraft).
- **FDM Update**: Received wind is converted to feet-per-second, inverted, and written directly to JSBSim's built-in wind property:
  `_fdmexec->SetPropertyValue("atmosphere/wind-down-fps", wind_down_fps);`
- **Wind Injection Node**: The `thermal_field_node` calculates the cumulative wind `w_total` from the active thermal field at the UAV's location and sends it to the respective UDP port at `5 Hz`.

---

## 4. Verification & Automated Test Logs

We ran [verify_env.py](file:///home/px4_sitl/sim_paper/scratch/verify_env.py) to validate all requirements.

### Test 1: Determinism
The generator nodes were launched twice with identical seeds. The generated coordinates and thermal profiles were logged and compared.
- **Result**: **PASSED**. Diffs were zero; same seeds yielded identical thermal coordinates and drift angles.

### Test 2: Wind Injection & FDM Response
The UAV took off to 50m AGL, entered offboard mode, and set throttle to `0.0` (gliding flight).
- **Baseline Sink Rate**: `~0.28 m/s`.
- **Updraft Applied**: Injecting a `+5.0 m/s` updraft via UDP port `15001`.
- **Result**: The vehicle's sink rate reversed into a positive climb rate of `+4.86 m/s` (NED velocity `vz = -4.86 m/s`). This proves that the vertical wind successfully reached the FDM and acted on the soaring physics of the glider.

### Test 3: Battery Estimation
- **Powered Climb Mode**: Power draw was positive (ranging from `~20 W` to `~84 W`), and battery SoC decreased monotonically.
- **Gliding Mode**: Power draw dropped to exactly `0.0 W`, and battery SoC remained flat.
- **Result**: **PASSED**. Monotonic decrease under power, flat during unpowered glides.

### Test 4: FOV Detection Checks
A dummy ground event was published directly underneath the gliding UAV.
- **Inside Cone**: When placed at the UAV's horizontal position, it registered a hit on `/soarer/fov/px4_1` (range `80.7 m` with confidence `0.97`).
- **Outside Cone**: When shifted beyond `alt * tan(30.0)` meters, the detection successfully cleared.
- **Result**: **PASSED**. Geometry checks correctly isolate targets inside the camera cone.

