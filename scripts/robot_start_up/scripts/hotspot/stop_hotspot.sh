#!/bin/bash

echo "Stopping robot services..."

# Load config from /etc or local fallback
CONFIG_FILE="/etc/robot_start_up/.env"
LOCAL_CONFIG_FILE="$(dirname "$0")/.env"

if [ -f "$CONFIG_FILE" ]; then
    echo "using installed env file"
    source "$CONFIG_FILE"
elif [ -f "$LOCAL_CONFIG_FILE" ]; then
    echo "using local env file"
    source "$LOCAL_CONFIG_FILE"
else
    echo "No .env config found. Assuming default connection ID: robot-hotspot"
    HOTSPOT_CONN_ID="robot-hotspot"
fi

# Stop Wi-Fi hotspot
echo "Stopping Wi-Fi connection: $HOTSPOT_CONN_ID"
nmcli con down "$HOTSPOT_CONN_ID" 2>/dev/null || echo "Hotspot '$HOTSPOT_CONN_ID' was not active or not found."

echo "Hotspot stopped."