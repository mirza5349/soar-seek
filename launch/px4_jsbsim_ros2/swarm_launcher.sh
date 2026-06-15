#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Configuration
WORKSPACE_DIR="/home/px4_sitl/sim_paper"
PX4_DIR="$WORKSPACE_DIR/PX4-Autopilot"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
AGENT_BIN="$WORKSPACE_DIR/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent"
PID_FILE="/tmp/soarer_swarm.pids"
ROS_DOMAIN=10
DDS_PORT=8889

# Default to 2 instances if not provided
N=${1:-2}

echo "================================================================="
echo " Starting Soarer Swarm Simulation Launcher (N = $N vehicles)"
echo " Dedicated ROS_DOMAIN_ID = $ROS_DOMAIN"
echo " Dedicated DDS Port      = $DDS_PORT"
echo "================================================================="

# Step 1: Ensure workspace paths
if [ ! -d "$PX4_DIR" ]; then
    echo "Error: PX4-Autopilot directory not found at $PX4_DIR"
    exit 1
fi

# Step 2: Build PX4 and JSBSim Bridge (Incrementally, without running)
echo "Step 1/5: Compiling simulation targets..."
pushd "$PX4_DIR" >/dev/null
DONT_RUN=1 make px4_sitl jsbsim_soarer
popd >/dev/null

# Step 3: Ensure MicroXRCEAgent is compiled
if [ ! -f "$AGENT_BIN" ]; then
    echo "DDS Agent binary not found at $AGENT_BIN. Rebuilding..."
    mkdir -p "$WORKSPACE_DIR/Micro-XRCE-DDS-Agent/build"
    pushd "$WORKSPACE_DIR/Micro-XRCE-DDS-Agent/build" >/dev/null
    cmake .. && make -j$(nproc)
    popd >/dev/null
fi

# Step 4: Verify ports are clear
echo "Step 2/5: Checking for network conflicts..."
CONFLICTS=0
# Check DDS port
if ss -tulpn | grep -q ":$DDS_PORT "; then
    # Check if it's our own agent already running
    if ps aux | grep -v grep | grep -q "MicroXRCEAgent.*-p $DDS_PORT"; then
        echo "   DDS Agent is already running on port $DDS_PORT."
    else
        echo "   Error: Port $DDS_PORT is already in use by another process!"
        CONFLICTS=1
    fi
fi

# Check simulation ports
for ((i=1; i<=N; i++)); do
    SIM_PORT=$((4560 + i))
    GCS_PORT=$((14550 + i))
    SDK_PORT=$((14540 + i))
    GCS_LOCAL=$((18570 + i))
    SDK_LOCAL=$((14580 + i))
    
    for PORT in $SIM_PORT $GCS_PORT $SDK_PORT $GCS_LOCAL $SDK_LOCAL; do
        if ss -tulpn | grep -q ":$PORT "; then
            echo "   Error: Port $PORT (instance $i) is already in use!"
            CONFLICTS=1
        fi
    done
done

if [ $CONFLICTS -eq 1 ]; then
    echo "Network conflicts detected. Please run './swarm_teardown.sh' first."
    exit 1
fi

# Clean up existing PID file
rm -f "$PID_FILE"

# Step 5: Start the Micro-XRCE-DDS Agent
if ! ps aux | grep -v grep | grep -q "MicroXRCEAgent.*-p $DDS_PORT"; then
    echo "Step 3/5: Starting Micro-XRCE-DDS Agent on UDP port $DDS_PORT..."
    nohup "$AGENT_BIN" udp4 -p "$DDS_PORT" > "$WORKSPACE_DIR/MicroXRCEAgent.log" 2>&1 &
    AGENT_PID=$!
    echo "$AGENT_PID" >> "$PID_FILE"
    echo "   Agent launched with PID: $AGENT_PID"
    sleep 1
else
    echo "Step 3/5: Micro-XRCE-DDS Agent already running."
fi

# Step 6: Spawn instances
echo "Step 4/5: Spawning $N vehicle simulation instances..."
for ((i=1; i<=N; i++)); do
    echo "   -> Launching UAV $i..."
    
    # Port configuration
    SIM_PORT=$((4560 + i))
    GCS_PORT=$((14550 + i))
    SDK_PORT=$((14540 + i))
    
    # Establish rootfs directory
    ROOTFS_DIR="$BUILD_DIR/rootfs_$i"
    rm -rf "$ROOTFS_DIR"
    mkdir -p "$ROOTFS_DIR"
    
    # Start jsbsim_bridge
    # jsbsim_bridge listens on TCP port 4560+i
    echo "      [UAV $i] Starting jsbsim_bridge on TCP port $SIM_PORT..."
    PX4_INSTANCE=$i nohup "$BUILD_DIR/build_jsbsim_bridge/jsbsim_bridge" soarer \
        -s "$PX4_DIR/Tools/simulation/jsbsim/jsbsim_bridge/scene/LSZH.xml" \
        > "$ROOTFS_DIR/jsbsim_bridge_$i.log" 2>&1 &
    BRIDGE_PID=$!
    echo "$BRIDGE_PID" >> "$PID_FILE"
    
    # Sleep to allow bridge to start up and bind to socket
    sleep 1.5
    
    # Start PX4 SITL inside its rootfs directory
    echo "      [UAV $i] Starting PX4 SITL (instance $i, GCS:$GCS_PORT, SDK:$SDK_PORT)..."
    pushd "$ROOTFS_DIR" >/dev/null
    
    # Export environment variables for isolation
    export PX4_INSTANCE=$i
    export PX4_SIM_MODEL=jsbsim_soarer
    export PX4_SIM_WORLD=LSZH
    export JSBSIM_AIRCRAFT_MODEL=soarer
    export PX4_UXRCE_DDS_PORT=$DDS_PORT
    export ROS_DOMAIN_ID=$ROS_DOMAIN
    
    nohup "$BUILD_DIR/bin/px4" -d -i $i "$BUILD_DIR/etc" > "px4_$i.log" 2>&1 &
    PX4_PID=$!
    echo "$PX4_PID" >> "$PID_FILE"
    
    popd >/dev/null
    
    echo "      [UAV $i] Spawned Bridge (PID: $BRIDGE_PID), PX4 (PID: $PX4_PID)"
done

# Step 7: Launch environment and estimation nodes
echo "Step 5/6: Starting environmental and sensing nodes..."
source "/opt/ros/humble/setup.bash"
pushd "$WORKSPACE_DIR/ros2_ws" >/dev/null
colcon build --packages-select soarer_msgs soarer_env
source "$WORKSPACE_DIR/ros2_ws/install/setup.bash"
popd >/dev/null
export ROS_DOMAIN_ID=$ROS_DOMAIN
nohup ros2 launch soarer_env launch_env.py num_vehicles:=$N launch_metrics:=${LAUNCH_METRICS:-true} > "$WORKSPACE_DIR/ros2_ws/soarer_env.log" 2>&1 &
ENV_LAUNCH_PID=$!
echo "$ENV_LAUNCH_PID" >> "$PID_FILE"
echo "   Environment nodes launcher started with PID: $ENV_LAUNCH_PID"

echo "Step 6/6: Swarm boot initialization complete."
echo "Active PIDs written to $PID_FILE"
echo "All logs are located under $BUILD_DIR/rootfs_[i]/"
echo "Use 'ros2 topic list' under ROS_DOMAIN_ID=$ROS_DOMAIN to check topics."
echo "================================================================="

