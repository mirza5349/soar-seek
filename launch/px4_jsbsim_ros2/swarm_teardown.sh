#!/usr/bin/env bash

# Configuration
PID_FILE="/tmp/soarer_swarm.pids"
WORKSPACE_DIR="/home/px4_sitl/sim_paper"

echo "================================================================="
echo " Stopping Soarer Swarm Simulation..."
echo "================================================================="

# Step 1: Kill processes by PID file
if [ -f "$PID_FILE" ]; then
    echo "Killing processes from PID file: $PID_FILE..."
    PIDS=$(cat "$PID_FILE")
    
    # Send SIGTERM first
    for PID in $PIDS; do
        if ps -p "$PID" >/dev/null 2>&1; then
            echo "   Sending SIGTERM to process $PID..."
            kill -15 "$PID" 2>/dev/null || true
        fi
    done
    
    sleep 2
    
    # Send SIGKILL for remaining processes
    for PID in $PIDS; do
        if ps -p "$PID" >/dev/null 2>&1; then
            echo "   Sending SIGKILL to process $PID..."
            kill -9 "$PID" 2>/dev/null || true
        fi
    done
    
    rm -f "$PID_FILE"
else
    echo "No PID file found at $PID_FILE."
fi

# Step 2: Fallback path-based cleanup
echo "Performing fallback check to clean up dangling soarer processes..."

# Gather all dangling PIDs
BRIDGE_PIDS=$(pgrep -f "$WORKSPACE_DIR.*/jsbsim_bridge" || true)
PX4_PIDS=$(pgrep -f "$WORKSPACE_DIR.*/bin/px4" || true)
AGENT_PIDS=$(pgrep -f "$WORKSPACE_DIR.*/MicroXRCEAgent" || true)
ENV_PIDS=$(pgrep -f "soarer_env" || true)
MAVSDK_PIDS=$(pgrep -f "mavsdk_server" || true)

ALL_DANGLING="$BRIDGE_PIDS $PX4_PIDS $AGENT_PIDS $ENV_PIDS $MAVSDK_PIDS"

if [ -n "$(echo $ALL_DANGLING | tr -d ' ')" ]; then
    echo "   Sending SIGTERM to dangling processes: $ALL_DANGLING"
    kill -15 $ALL_DANGLING 2>/dev/null || true
    sleep 3
    
    # Verify and kill -9 if still running
    STILL_RUNNING=""
    for PID in $ALL_DANGLING; do
        if ps -p "$PID" >/dev/null 2>&1; then
            STILL_RUNNING="$STILL_RUNNING $PID"
        fi
    done
    if [ -n "$(echo $STILL_RUNNING | tr -d ' ')" ]; then
        echo "   Sending SIGKILL to remaining processes: $STILL_RUNNING"
        kill -9 $STILL_RUNNING 2>/dev/null || true
    fi
fi


echo "Teardown complete. Verifying ports..."
ss -tulpn | grep -E "456|1455|1454|8889" || echo "   All ports cleared!"

echo "================================================================="
