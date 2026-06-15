#!/usr/bin/env bash

# Soarer JSBSim/PX4 SITL Run Script (Step 2)
# Configures port isolation (instance 1) and headless execution.

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PX4_DIR="${SCRIPT_DIR}/PX4-Autopilot"

echo "=== Launching Soarer PX4 SITL Simulation ==="
echo "Port Isolation: PX4_INSTANCE=1 (TCP 4561, MAVLink GCS 14551)"
echo "Mode: HEADLESS=1 (No FlightGear GUI)"
echo "Working directory: ${PX4_DIR}"
echo "============================================"

# Ensure disjoint environment
unset GZ_IP GZ_PARTITION
export PX4_INSTANCE=1
export HEADLESS=1

# Compile and run
cd "${PX4_DIR}"
make px4_sitl jsbsim_soarer
