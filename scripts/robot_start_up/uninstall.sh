#!/bin/bash

echo "Uninstalling robot services..."

SERVICES=(
  robot-hotspot.service
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

# Reload systemd to apply changes
sudo systemctl daemon-reexec

#Remove entire config folder  
sudo rm -rf /etc/robot_start_up

echo "Uninstall complete."
