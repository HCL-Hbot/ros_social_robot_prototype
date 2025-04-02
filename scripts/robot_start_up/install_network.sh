### install_network.sh
#!/bin/bash

echo "Installing hotspot network service..."

INSTALL_DIR="/usr/local/bin"
BASE_DIR="$(cd "$(dirname "$0")"; pwd)"
SCRIPT_SRC="$BASE_DIR/scripts"
SERVICE_SRC="$BASE_DIR/services"
ROBOT_USER=$(logname)

ENV_TARGET="/etc/robot_start_up"

# Set up .env file
if [ -f "$SCRIPT_SRC/.env" ]; then
    sudo mkdir -p "$ENV_TARGET"
    echo "Using your local .env file"
    sudo cp "$SCRIPT_SRC/.env" "$ENV_TARGET/.env"
elif [ -f "$SCRIPT_SRC/.env.example" ]; then
    sudo mkdir -p "$ENV_TARGET"
    echo "No .env found — using .env.example as fallback"
    sudo cp "$SCRIPT_SRC/.env.example" "$ENV_TARGET/.env"
else
    echo "No .env or .env.example found in $SCRIPT_SRC. Cannot configure hotspot."
    exit 1
fi

# Load .env to get hotspot ID
source "$ENV_TARGET/.env"

# Set proper permissions
sudo chown root:root "$ENV_TARGET/.env"
sudo chmod 600 "$ENV_TARGET/.env"


HOTSPOT_CONN_ID="${HOTSPOT_CONN_ID:-robot-hotspot}"

# Disable autoconnect for all other Wi-Fi profiles
echo "Disabling autoconnect for all Wi-Fi profiles except $HOTSPOT_CONN_ID..."
for UUID in $(nmcli -t -f UUID,TYPE con show | grep '^.*:wifi$' | cut -d: -f1); do
    CUR_ID=$(nmcli -g NAME con show "$UUID")
    if [ "$CUR_ID" != "$HOTSPOT_CONN_ID" ]; then
        nmcli connection modify "$UUID" connection.autoconnect no || true
    fi
done

# Activate hotspot systemd service
echo "Activating hotspot systemd service..."
sudo cp "$SERVICE_SRC/robot-hotspot.service" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot-hotspot.service

echo "Network hotspot service installed."