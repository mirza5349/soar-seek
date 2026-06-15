#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# launch_gz_x500.sh  —  PX4 SITL + Gazebo Harmonic (X500 quadrotor)
# ─────────────────────────────────────────────────────────────────────────────
# Usage:
#   ./launch_gz_x500.sh              # interactive Gazebo GUI
#   ./launch_gz_x500.sh --headless   # headless (no window), useful for CI
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PX4_DIR="${SCRIPT_DIR}/PX4-Autopilot"
PX4_BIN="${PX4_DIR}/build/px4_sitl_default/bin/px4"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"

HEADLESS=0
if [[ "${1:-}" == "--headless" ]]; then
    HEADLESS=1
fi

# ── Sanity checks ─────────────────────────────────────────────────────────────
if [[ ! -x "$PX4_BIN" ]]; then
    echo "[gz_x500] PX4 binary not found at $PX4_BIN"
    echo "          Run:  cd ${PX4_DIR} && make px4_sitl_default"
    exit 1
fi

if ! command -v gz &>/dev/null; then
    echo "[gz_x500] Gazebo Harmonic (gz) not found. Install: sudo apt install gz-harmonic"
    exit 1
fi

# ── Kill any stale simulation processes ──────────────────────────────────────
echo "[gz_x500] Clearing any previous simulation processes…"
pkill -f "gz sim"        2>/dev/null || true
pkill -f "px4 "          2>/dev/null || true
sleep 1

# ── Environment ───────────────────────────────────────────────────────────────
export DISPLAY="${DISPLAY:-:1}"
export GZ_SIM_RESOURCE_PATH="${PX4_DIR}/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
export PX4_SIM_MODEL="gz_x500"
export PX4_GZ_WORLD="default"
export PX4_HOME_LAT="${PX4_HOME_LAT:-47.397742}"
export PX4_HOME_LON="${PX4_HOME_LON:-8.545594}"
export PX4_HOME_ALT="${PX4_HOME_ALT:-488.0}"

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║   PX4 SITL  +  Gazebo Harmonic  —  X500 Quadrotor           ║"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Model  : X500 (quadrotor)                                   ║"
echo "║  World  : default                                            ║"
echo "║  MAVLink: UDP  localhost:14540 (QGC: 14550)                 ║"
echo "║  Headless: $([ $HEADLESS -eq 1 ] && echo 'YES' || echo 'NO ')                                             ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# ── Launch Gazebo ─────────────────────────────────────────────────────────────
GZ_WORLD_FILE="${PX4_DIR}/Tools/simulation/gz/worlds/${PX4_GZ_WORLD}.sdf"
GZ_ARGS="--ros-args"

if [[ $HEADLESS -eq 1 ]]; then
    echo "[gz_x500] Starting Gazebo headless…"
    GZ_SERVER_ARGS="-s"
    DISPLAY= gz sim -s "${GZ_WORLD_FILE}" &
else
    echo "[gz_x500] Starting Gazebo GUI…"
    DISPLAY="${DISPLAY}" gz sim "${GZ_WORLD_FILE}" &
fi
GZ_PID=$!
echo "[gz_x500] Gazebo PID: $GZ_PID"

# Wait for Gazebo to initialise
echo "[gz_x500] Waiting for Gazebo to initialise (5 s)…"
sleep 5

# ── Launch PX4 SITL ───────────────────────────────────────────────────────────
echo "[gz_x500] Starting PX4 SITL…"
cd "${BUILD_DIR}"
"${PX4_BIN}" "${BUILD_DIR}/etc" -s etc/init.d-posix/rcS &
PX4_PID=$!
echo "[gz_x500] PX4 PID: $PX4_PID"

echo ""
echo "[gz_x500] ✓ Simulation running. Press Ctrl+C to stop."
echo "[gz_x500]   QGroundControl: connect UDP  localhost:14550"
echo "[gz_x500]   MAVLink API   : localhost:14540"
echo ""

# ── Cleanup on exit ───────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "[gz_x500] Shutting down…"
    kill "$PX4_PID" 2>/dev/null || true
    kill "$GZ_PID"  2>/dev/null || true
    pkill -f "gz sim"  2>/dev/null || true
    pkill -f "px4 "    2>/dev/null || true
    echo "[gz_x500] Done."
}
trap cleanup INT TERM EXIT

wait "$PX4_PID"
