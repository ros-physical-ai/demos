#!/bin/bash
# Wrapper script to ensure Zenoh router is running before executing ROS 2 commands

set -e

ZENOH_STARTED_BY_SCRIPT=false
ZENOH_PID=""

# Cleanup function to kill Zenoh if we started it
cleanup() {
    if [ "$ZENOH_STARTED_BY_SCRIPT" = true ] && [ -n "$ZENOH_PID" ]; then
        if kill -0 "$ZENOH_PID" 2>/dev/null; then
            echo "Stopping Zenoh router (PID: $ZENOH_PID)..."
            kill "$ZENOH_PID" 2>/dev/null || true
        fi
    fi
}

# Set up signal handlers to cleanup on exit
trap cleanup EXIT INT TERM

# Check if Zenoh router (rmw_zenohd) is already running
if ! pgrep -f "rmw_zenohd" > /dev/null; then
    echo "Zenoh router not detected. Starting rmw_zenohd in background..."
    # Start Zenoh router in background
    ros2 run rmw_zenoh_cpp rmw_zenohd > /dev/null 2>&1 &
    ZENOH_PID=$!
    ZENOH_STARTED_BY_SCRIPT=true
    
    # Wait a moment for Zenoh to initialize
    sleep 2
    
    # Verify it's actually running
    if ! pgrep -f "rmw_zenohd" > /dev/null; then
        echo "ERROR: Failed to start Zenoh router"
        exit 1
    fi
    
    echo "Zenoh router started (PID: $ZENOH_PID)"
else
    echo "Zenoh router already running"
fi

# Execute the command passed as arguments
"$@"

