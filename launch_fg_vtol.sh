#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# launch_fg_vtol.sh  —  PX4 SITL + FlightGear (TF-G1 autogyro VTOL)
# ─────────────────────────────────────────────────────────────────────────────
# Usage:
#   ./launch_fg_vtol.sh              # TF-G1 autogyro (default)
#   ./launch_fg_vtol.sh tf-g2        # TF-G2 autogyro (larger)
#   ./launch_fg_vtol.sh rascal       # Rascal fixed-wing
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PX4_DIR="${SCRIPT_DIR}/PX4-Autopilot"

FG_MODEL="${1:-tf-g1}"
VALID_MODELS="tf-g1 tf-g2 rascal rascal-electric tf-r1"

# ── Validate model ─────────────────────────────────────────────────────────────
if ! echo "$VALID_MODELS" | grep -wq "$FG_MODEL"; then
    echo "[fg_vtol] Unknown model: $FG_MODEL"
    echo "          Valid models: $VALID_MODELS"
    exit 1
fi

# ── Sanity checks ─────────────────────────────────────────────────────────────
# fgfs lives in /usr/games which may not be in PATH — add it
export PATH="/usr/games:${PATH}"

if ! command -v fgfs &>/dev/null; then
    echo "[fg_vtol] FlightGear (fgfs) not found."
    echo "          Install: sudo apt install flightgear"
    exit 1
fi

FGFS_VERSION=$(fgfs --version 2>&1 | head -1)
echo "[fg_vtol] FlightGear: $FGFS_VERSION"

# ── Fix FlightGear Protocol directory permissions ──────────────────────────────
FG_ROOT=$(fgfs --version 2>&1 | grep FG_ROOT | awk -F= '{print $2}' | tr -d ' ')
if [[ -z "$FG_ROOT" ]]; then
    # Common fallback paths
    for p in /usr/share/games/flightgear /usr/share/FlightGear /snap/flightgear/current/usr/share/games/flightgear; do
        if [[ -d "$p" ]]; then FG_ROOT="$p"; break; fi
    done
fi

PROTOCOL_DIR="${FG_ROOT}/Protocol"
if [[ -d "$PROTOCOL_DIR" ]] && [[ ! -w "$PROTOCOL_DIR" ]]; then
    echo "[fg_vtol] Protocol dir not writable: $PROTOCOL_DIR"
    echo "[fg_vtol] Fixing permissions (requires sudo)…"
    pkexec chmod a+w "$PROTOCOL_DIR" || sudo chmod a+w "$PROTOCOL_DIR"
fi

# ── Kill any stale simulation processes ──────────────────────────────────────
echo "[fg_vtol] Clearing any previous simulation processes…"
pkill -f "fgfs"          2>/dev/null || true
pkill -f "flightgear_bridge" 2>/dev/null || true
pkill -f "px4 "          2>/dev/null || true
sleep 1

# ── Environment ───────────────────────────────────────────────────────────────
export DISPLAY="${DISPLAY:-:1}"
export PX4_SIM_MODEL="flightgear_${FG_MODEL}"
export PX4_HOME_LAT="${PX4_HOME_LAT:-47.397742}"
export PX4_HOME_LON="${PX4_HOME_LON:-8.545594}"
export PX4_HOME_ALT="${PX4_HOME_ALT:-488.0}"

# Model-specific airframe label
case "$FG_MODEL" in
    tf-g1)  MODEL_LABEL="ThunderFly TF-G1 Autogyro VTOL" ;;
    tf-g2)  MODEL_LABEL="ThunderFly TF-G2 Autogyro VTOL" ;;
    rascal) MODEL_LABEL="Rascal Fixed-Wing Plane" ;;
    rascal-electric) MODEL_LABEL="Rascal Electric Fixed-Wing" ;;
    tf-r1)  MODEL_LABEL="TF-R1 Ground Rover" ;;
    *)      MODEL_LABEL="$FG_MODEL" ;;
esac

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║   PX4 SITL  +  FlightGear  —  ${MODEL_LABEL}"
echo "╠══════════════════════════════════════════════════════════════╣"
echo "║  Model  : $FG_MODEL"
echo "║  MAVLink: UDP  localhost:14540 (QGC: 14550)                 ║"
echo "║  FG Root: ${FG_ROOT:-unknown}"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# ── Build nolockstep PX4 + FlightGear bridge (if not yet built) ───────────────
NOLOCKSTEP_BIN="${PX4_DIR}/build/px4_sitl_nolockstep/bin/px4"
BRIDGE_BIN="${PX4_DIR}/build/px4_sitl_nolockstep/build_flightgear_bridge/flightgear_bridge"

if [[ ! -x "$NOLOCKSTEP_BIN" ]]; then
    echo "[fg_vtol] Building PX4 nolockstep (this takes ~4 min on first run)…"
    cd "${PX4_DIR}"
    make px4_sitl_nolockstep 2>&1 | tail -5
fi

if [[ ! -x "$BRIDGE_BIN" ]]; then
    echo "[fg_vtol] Building FlightGear bridge…"
    cd "${PX4_DIR}"
    cmake --build build/px4_sitl_nolockstep --target flightgear_bridge -j$(nproc) 2>&1 | tail -5
    echo "[fg_vtol] ✓ Bridge ready: $BRIDGE_BIN"
fi

# ── Export bridge binary path so sitl_run.sh can find it ──────────────────────
export FG_BINARY="${FG_BINARY:-fgfs}"

# ── Run via PX4 FlightGear sitl_run.sh ────────────────────────────────────────
PX4_BIN="${PX4_DIR}/build/px4_sitl_nolockstep/bin/px4"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_nolockstep"
ROOTFS="${BUILD_DIR}/rootfs"
mkdir -p "${ROOTFS}"

cd "${PX4_DIR}/Tools/simulation/flightgear/flightgear_bridge"

echo "[fg_vtol] Launching FlightGear with model: $FG_MODEL…"
python3 FG_run.py "models/${FG_MODEL}.json" 0 &
FG_PID=$!
echo "[fg_vtol] FlightGear PID: $FG_PID"

echo "[fg_vtol] Starting FlightGear bridge…"
"${BRIDGE_BIN}" 0 $(python3 get_FGbridge_params.py "models/${FG_MODEL}.json") &
BRIDGE_PID=$!
echo "[fg_vtol] Bridge PID: $BRIDGE_PID"

echo "[fg_vtol] Waiting for FlightGear to fully load (20 s)…"
sleep 20

echo "[fg_vtol] Starting PX4 SITL (nolockstep)…"
cd "${ROOTFS}"
"${PX4_BIN}" "${BUILD_DIR}/etc" &
PX4_PID=$!
echo "[fg_vtol] PX4 PID: $PX4_PID"

echo ""
echo "[fg_vtol] ✓ Simulation running. Press Ctrl+C to stop."
echo "[fg_vtol]   QGroundControl: connect UDP  localhost:14550"
echo "[fg_vtol]   MAVLink API   : localhost:14540"
echo ""

# ── Cleanup on exit ───────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "[fg_vtol] Shutting down…"
    kill "$PX4_PID"    2>/dev/null || true
    kill "$BRIDGE_PID" 2>/dev/null || true
    kill "$FG_PID"     2>/dev/null || true
    pkill -f "fgfs"             2>/dev/null || true
    pkill -f "flightgear_bridge" 2>/dev/null || true
    # Clean up PID file left by FG_run.py
    rm -f /tmp/px4fgfspid_0
    echo "[fg_vtol] Done."
}
trap cleanup INT TERM EXIT

wait "$PX4_PID"
