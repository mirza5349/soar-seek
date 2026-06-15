# Vendored-repository patches

The upstream simulators (`PX4-Autopilot`, `jsbsim_src`, `Micro-XRCE-DDS-Agent`)
are large external clones and are **not** committed to this repository. The
project-specific modifications made to them are captured here as patch files so
the build can be reproduced.

## `px4_autopilot_soarer.patch`
Apply inside a matching `PX4-Autopilot` checkout:

```bash
cd PX4-Autopilot
git apply ../patches/px4_autopilot_soarer.patch
```

It contains the Soar & Seek modifications:
- `Tools/simulation/jsbsim/jsbsim_bridge/` — emit `HIL_STATE_QUATERNION` ground
  truth from the JSBSim FDM (for PX4-EKF-vs-JSBSim verification).
- `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` — unique offboard remote
  port (`14640+instance`) so UAV IDs >= 10 arm correctly.
- `src/modules/uxrce_dds_client/dds_topics.yaml` — expose
  `vehicle_local_position_groundtruth` and `vehicle_attitude_groundtruth`.

Rebuild after applying: `DONT_RUN=1 make px4_sitl` and rebuild the jsbsim bridge.
