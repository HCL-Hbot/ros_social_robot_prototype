#!/bin/bash

echo "Installing robot services..."

INSTALL_DIR="/usr/local/bin"
BASE_DIR="$(cd "$(dirname "$0")"; pwd)"
SCRIPT_SRC="$BASE_DIR/scripts"
SERVICE_SRC="$BASE_DIR/services"
ROBOT_USER=$(logname)

# Make scripts executable
chmod +x "$SCRIPT_SRC"/*.sh

# Copy scripts to system path
echo "Copying scripts to $INSTALL_DIR..."
sudo cp "$SCRIPT_SRC"/*.sh "$INSTALL_DIR"


# Replace %USER% in all services except robot-hotspot (robot-hotspot is optional and need root privileges)
TEMP_DIR=$(mktemp -d)
for FILE in "$SERVICE_SRC"/*.service; do
  FILENAME="$(basename "$FILE")"
  if [[ "$FILENAME" != "robot-hotspot.service" ]]; then
    sed "s|%USER%|$ROBOT_USER|g" "$FILE" > "$TEMP_DIR/$FILENAME"
  fi
done

# Copy systemd services
echo "Installing systemd services..."
sudo cp "$TEMP_DIR"/*.service /etc/systemd/system/

# Enable services
echo "Enabling services..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable robot-ssh-init.service
sudo systemctl enable robot-display.service
#sudo systemctl enable robot-launch-eye.service

# Clean up temp
rm -r "$TEMP_DIR"

echo "Installation complete. Services will run on next boot."
