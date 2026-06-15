# Finite State Machine Autonomy (Step 4b)

This document describes the design, implementation, and interfaces of the decentralized Finite State Machine (FSM) running independently on each UAV.

---

## 1. FSM State Diagram

The FSM comprises 6 distinct operational states:

```mermaid
stateDiagram-v2
    [*] --> PATROL : Boot / Arm
    
    PATROL --> EVENT_INVESTIGATION : FOV Detection\n(Priority >= 3)
    EVENT_INVESTIGATION --> PATROL : Duration Met (30s)\nor Event Expired
    
    PATROL --> THERMAL_SEARCH : Low Battery (SoC <= 30%)\nor Swarm Thermal Cue
    EVENT_INVESTIGATION --> THERMAL_SEARCH : Investigation Met\n& SoC <= 30%
    
    THERMAL_SEARCH --> THERMAL_EXPLOITATION : Usable Lift Found\n(w_i >= 1.5 m/s)
    THERMAL_SEARCH --> PATROL : Search Timeout (45s)
    
    THERMAL_EXPLOITATION --> GLIDE_RETURN : Ceiling Reached (150m)\nor Lift Lost (> 5s)
    
    GLIDE_RETURN --> PATROL : Rejoined Route (< 50m)\nor Low Alt (< 40m)
    
    PATROL --> LANDING : Critical SoC (SoC <= 7%)
    EVENT_INVESTIGATION --> LANDING : Critical SoC (SoC <= 7%)
    THERMAL_SEARCH --> LANDING : Critical SoC (SoC <= 7%)
    THERMAL_EXPLOITATION --> LANDING : Critical SoC (SoC <= 7%)
    GLIDE_RETURN --> LANDING : Critical SoC (SoC <= 7%)
    
    LANDING --> [*] : Touchdown / Disarm
```

---

## 2. State & Transition Descriptions

| State | Controller / MAVSDK Command | Transition Triggers | Action on Transition |
|---|---|---|---|
| **PATROL** | Mission Plugin (`start_mission`) | 1. `SoC <= 30%` -> `THERMAL_SEARCH`<br>2. High-priority FOV event -> `EVENT_INVESTIGATION`<br>3. Swarm `ThermalCue` received -> `THERMAL_SEARCH`<br>4. `SoC <= 7%` -> `LANDING` | `upload_patrol_mission`, resume waypoint path |
| **EVENT_INVESTIGATION** | Action Plugin (`do_orbit`) | 1. Duration met (30s) -> `PATROL`<br>2. Event expired -> `PATROL`<br>3. `SoC <= 7%` -> `LANDING` | Orbit event location with 50m radius |
| **THERMAL_SEARCH** | Action Plugin (`goto_location`) | 1. Usable lift ($w_i \ge 1.5$ m/s) -> `THERMAL_EXPLOITATION`<br>2. Timeout (45s) -> `PATROL`<br>3. `SoC <= 7%` -> `LANDING` | Fly towards core center location |
| **THERMAL_EXPLOITATION** | Offboard Plugin (`set_attitude` with thrust = 0.0) | 1. Rel Alt $\ge 150$ m -> `GLIDE_RETURN`<br>2. Lift lost ($w_i < 1.5$ m/s for > 5s) -> `GLIDE_RETURN`<br>3. `SoC <= 7%` -> `LANDING` | Circle core (roll = 30°, pitch = 2°), publish `ThermalCue` |
| **GLIDE_RETURN** | Offboard Plugin (`set_attitude` with thrust = 0.0) | 1. Distance to WP < 50m -> `PATROL`<br>2. Rel Alt < 40m -> `PATROL`<br>3. `SoC <= 7%` -> `LANDING` | Glide straight to next patrol waypoint |
| **LANDING** | Mission Plugin (`start_mission`) | UAV landed (`in_air == False`) -> Disarm | Upload 3-point landing mission down to runway |

---

## 3. Decentralized ROS 2 Topic Map (ROS_DOMAIN_ID = 10)

The following new messages were added to the `soarer_msgs` package and are utilized swarm-wide:

| Topic Name | Message Type | Pub / Sub | Description |
|---|---|---|---|
| `/soarer/fsm/px4_{i}` | `soarer_msgs/msg/FsmState` | Pub | Current FSM state ID and name |
| `/soarer/telemetry/px4_{i}` | `soarer_msgs/msg/TelemetryExchange` | Pub | GPS position, Alt, NED vel, and FSM state |
| `/soarer/thermal_cues` | `soarer_msgs/msg/ThermalCue` | Pub/Sub | Shared coordinates of active thermals being exploited |

---

## 4. Verification and Acceptance Logs

We verified all behaviors using the automated test suite `scratch/verify_env.py`.
Logs and result summaries are written to `ENV.md` and `walkthrough.md`.
All tests passed successfully, confirming state-driven power draw closure, safety constraints, multi-vehicle isolation, and automated landings.
