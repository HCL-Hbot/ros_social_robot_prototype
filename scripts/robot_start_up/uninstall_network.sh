### uninstall_network.sh
#!/bin/bash

echo "Uninstalling hotspot service..."

HOTSPOT_ID="robot-hotspot"
CONFIG_FILE="/etc/robot_start_up/.env"
BIN_DIR="/usr/local/bin"
SCRIPT_NAMES=(
   start_hotspot.sh
   stop_hotspot.sh
)

if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    HOTSPOT_ID="${HOTSPOT_CONN_ID:-robot-hotspot}"
fi

# Stop and disable service
sudo systemctl stop robot-hotspot.service || true
sudo systemctl disable robot-hotspot.service || true
sudo rm -f /etc/systemd/system/robot-hotspot.service
sudo systemctl daemon-reload

# Remove hotspot connection
echo "Removing hotspot connection: $HOTSPOT_ID"
nmcli con delete "$HOTSPOT_ID" || echo "Connection $HOTSPOT_ID not found."

# Re-enable autoconnect for all Wi-Fi profiles
echo "Re-enabling autoconnect for all Wi-Fi profiles..."
for UUID in $(nmcli -t -f UUID,TYPE con show | grep '^.*:wifi$' | cut -d: -f1); do
    nmcli connection modify "$UUID" connection.autoconnect yes || true
done

#remove .env config file
sudo rm -rf /etc/robot_start_up

for script in "${SCRIPT_NAMES[@]}"; do
  sudo rm -f "$BIN_DIR/$script"
done

# Remove all installed scripts
echo "Hotspot network uninstalled."
