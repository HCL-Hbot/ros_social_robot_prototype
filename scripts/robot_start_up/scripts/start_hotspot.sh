#!/bin/bash

# Load environment config
CONFIG_FILE="/etc/robot_start_up/.env"
LOCAL_CONFIG_FILE="$(dirname "$0")/.env"

if [ -f "$CONFIG_FILE" ]; then
    echo "using installed env file"
    source "$CONFIG_FILE"    
elif [ -f "$LOCAL_CONFIG_FILE" ]; then
    echo "using local env file"
    source "$LOCAL_CONFIG_FILE"
else
    echo "Config file .env not found. Please create it in the same folder as this script."
    exit 1
fi

SSID="${HOTSPOT_SSID:-MyHotspot}"
PASSWORD="${HOTSPOT_PASSWORD:-changeme123}"
IFACE="${HOTSPOT_INTERFACE:-wlan0}"

# Custom connection ID (not 'Hotspot' to avoid GUI interference)
CONN_ID="${HOTSPOT_CONN_ID:-robot-hotspot}"


# Delete existing connection if it exists
if nmcli con show "$CONN_ID" &>/dev/null; then
    echo "Removing existing connection '$CONN_ID'"
    nmcli con delete "$CONN_ID"
fi

# Create new connection
echo "Creating hotspot '$SSID' on interface '$IFACE'"
nmcli con add type wifi ifname "$IFACE" con-name "$CONN_ID" autoconnect yes ssid "$SSID"
nmcli con modify "$CONN_ID" 802-11-wireless.mode ap 802-11-wireless.band bg ipv4.method shared
nmcli con modify "$CONN_ID" wifi-sec.key-mgmt wpa-psk
nmcli con modify "$CONN_ID" wifi-sec.psk "$PASSWORD"

# Start the hotspot
nmcli con up "$CONN_ID"