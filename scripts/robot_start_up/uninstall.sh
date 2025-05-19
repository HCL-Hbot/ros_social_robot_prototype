#!/bin/bash

set -e

echo "Uninstalling robot services..."

SERVICE_NAMES=(
  robot-display
  robot-eye
  robot-micro-ros-agent
  robot-interaction-controller
)

SYSTEMD_DIR="/etc/systemd/system"
BIN_DIR="/usr/local/bin"

# 1. Stop, disable, and remove systemd services
for name in "${SERVICE_NAMES[@]}"; do
  SERVICE_FILE="$name.service"
  echo "Disabling and removing $SERVICE_FILE..."
  sudo systemctl stop "$SERVICE_FILE" || true
  sudo systemctl disable "$SERVICE_FILE" || true
  sudo rm -f "$SYSTEMD_DIR/$SERVICE_FILE"
done

# 2. Remove all installed scripts
echo "Removing installed scripts from $BIN_DIR..."
SCRIPT_NAMES=(
  start_ssh.sh
  stop_ssh.sh
  rotate_screens.sh
)

for script in "${SCRIPT_NAMES[@]}"; do
  sudo rm -f "$BIN_DIR/$script"
done

# 3. Reload systemd
echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

# 4. Clean config folder
echo "Uninstallation complete."
