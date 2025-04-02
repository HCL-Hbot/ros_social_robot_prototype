#!/bin/bash

echo "Uninstalling robot services..."

SERVICES=(
  robot-ssh-init.service
  robot-display.service
  #robot-launch-eye.service
)

# Stop, disable, and remove each systemd service
for SERVICE in "${SERVICES[@]}"; do
  echo "Disabling and removing $SERVICE..."
  sudo systemctl stop "$SERVICE" 2>/dev/null
  sudo systemctl disable "$SERVICE" 2>/dev/null
  sudo rm -f "/etc/systemd/system/$SERVICE"
done

#Remove entire config folder  
sudo rm -rf /etc/robot_start_up

echo "Resetting autoconnect for all Wi-Fi profiles..."
for UUID in $(nmcli -t -f UUID,TYPE con show | grep '^.*:wifi$' | cut -d: -f1); do
    nmcli connection modify "$UUID" connection.autoconnect yes || true
done

# Reload systemd to apply changes
echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "Uninstall complete."
