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


echo "Recreating hotspot '$SSID' on interface '$IFACE'..."

# Delete existing connection
nmcli con delete "$CONN_ID" &>/dev/null || true

nmcli con add type wifi ifname "$IFACE" con-name "$CONN_ID" ssid "$SSID"
nmcli con modify "$CONN_ID" 802-11-wireless.mode ap
nmcli con modify "$CONN_ID" 802-11-wireless.band bg
nmcli con modify "$CONN_ID" ipv4.addresses 10.42.0.1/24
nmcli con modify "$CONN_ID" ipv4.gateway 10.42.0.1
nmcli con modify "$CONN_ID" ipv4.method shared
nmcli con modify "$CONN_ID" wifi-sec.key-mgmt wpa-psk
nmcli con modify "$CONN_ID" wifi-sec.psk "$PASSWORD"
nmcli con modify "$CONN_ID" 802-11-wireless-security.proto rsn
nmcli con modify "$CONN_ID" 802-11-wireless-security.group ccmp
nmcli con modify "$CONN_ID" 802-11-wireless-security.pairwise ccmp

echo "Disconnecting other Wi-Fi profiles..."
ACTIVE_CONNS=$(nmcli -t -f UUID,TYPE con show --active | grep '^.*:wifi$' | cut -d: -f1)
for UUID in $ACTIVE_CONNS; do
    CUR_ID=$(nmcli -g NAME con show "$UUID")
    if [ "$CUR_ID" != "$CONN_ID" ]; then
        echo "Disconnecting: $CUR_ID"
        nmcli con down "$CUR_ID" || true
    fi
done

echo "Starting hotspot '$SSID'..."
# Start the hotspot
nmcli con up "$CONN_ID"