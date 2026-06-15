#!/usr/bin/env python3
"""PX4-EKF vs JSBSim-FDM verification capture.

Launches an N-UAV swarm, logs the PX4 estimate (vehicle_local_position /
vehicle_attitude) alongside the simulator ground truth
(vehicle_local_position_groundtruth / vehicle_attitude_groundtruth) for each
UAV, then tears down. Raw per-sample rows -> logs/raw/verification/px4jsbsim_raw.csv
"""
import os
import sys
import csv
import time
import math
import subprocess

WS = "/home/px4_sitl/sim_paper"
OUT_DIR = os.path.join(WS, "logs/raw/verification")
N = int(sys.argv[1]) if len(sys.argv) > 1 else 6
DURATION = int(sys.argv[2]) if len(sys.argv) > 2 else 150


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    subprocess.run([os.path.join(WS, "swarm_teardown.sh")], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    env = dict(os.environ, LAUNCH_METRICS="false", ROS_DOMAIN_ID="10")
    subprocess.run([os.path.join(WS, "swarm_launcher.sh"), str(N)], check=True, env=env)
    print("Swarm up; settling 40 s before capture...")
    time.sleep(40)

    # The logger node runs in THIS process and must share the sim's ROS domain.
    os.environ["ROS_DOMAIN_ID"] = "10"
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude

    rclpy.init()
    node = Node("px4jsbsim_logger")
    est_pos, gt_pos, est_att, gt_att = {}, {}, {}, {}

    def mk(d, uav):
        return lambda m, u=uav: d.__setitem__(u, m)

    for i in range(1, N + 1):
        node.create_subscription(VehicleLocalPosition, f'/px4_{i}/fmu/out/vehicle_local_position',
                                 mk(est_pos, i), qos_profile_sensor_data)
        node.create_subscription(VehicleLocalPosition, f'/px4_{i}/fmu/out/vehicle_local_position_groundtruth',
                                 mk(gt_pos, i), qos_profile_sensor_data)
        node.create_subscription(VehicleAttitude, f'/px4_{i}/fmu/out/vehicle_attitude',
                                 mk(est_att, i), qos_profile_sensor_data)
        node.create_subscription(VehicleAttitude, f'/px4_{i}/fmu/out/vehicle_attitude_groundtruth',
                                 mk(gt_att, i), qos_profile_sensor_data)

    path = os.path.join(OUT_DIR, "px4jsbsim_raw.csv")
    f = open(path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["uav_id", "sim_time",
                "est_x", "est_y", "est_z", "est_vx", "est_vy", "est_vz",
                "gt_x", "gt_y", "gt_z", "gt_vx", "gt_vy", "gt_vz",
                "est_roll", "est_pitch", "est_yaw", "gt_roll", "gt_pitch", "gt_yaw",
                "est_ts_us", "gt_ts_us"])

    def quat_euler(q):  # q = [w,x,y,z]
        w_, x, y, z = q
        roll = math.atan2(2 * (w_ * x + y * z), 1 - 2 * (x * x + y * y))
        sp = 2 * (w_ * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sp)))
        yaw = math.atan2(2 * (w_ * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

    print(f"Capturing for {DURATION} s...")
    t0 = time.time()
    rows = 0
    while time.time() - t0 < DURATION:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        # sample at ~5 Hz wall when both est+gt present for a UAV
        if not hasattr(main, "_last") or now - main._last >= 0.2:
            main._last = now
            for i in range(1, N + 1):
                if i in est_pos and i in gt_pos and i in est_att and i in gt_att:
                    ep, gp, ea, ga = est_pos[i], gt_pos[i], est_att[i], gt_att[i]
                    er, epi, ey = quat_euler(ea.q)
                    gr, gpi, gy = quat_euler(ga.q)
                    w.writerow([i, round(ep.timestamp / 1e6, 3),
                                ep.x, ep.y, ep.z, ep.vx, ep.vy, ep.vz,
                                gp.x, gp.y, gp.z, gp.vx, gp.vy, gp.vz,
                                round(er, 5), round(epi, 5), round(ey, 5),
                                round(gr, 5), round(gpi, 5), round(gy, 5),
                                ep.timestamp, gp.timestamp])
                    rows += 1
    f.close()
    node.destroy_node()
    rclpy.shutdown()
    print(f"Captured {rows} rows -> {path}")
    subprocess.run([os.path.join(WS, "swarm_teardown.sh")], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
