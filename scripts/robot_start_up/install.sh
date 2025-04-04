#!/bin/bash

set -e

SYSTEMD_DIR="/etc/systemd/system"
INSTALL_DIR="/usr/local/bin"

BASE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_PATH=$(realpath "$BASE_DIR/../..")
USER_NAME=$(whoami)

# Find npm path (for Electron apps)
NPM_PATH=$(which npm || true)
if [ -z "$NPM_PATH" ]; then
  echo "❌ Error: npm is not installed or not in PATH. Please install Node.js and npm first."
  exit 1
fi
NPM_DIR=$(dirname "$NPM_PATH")

# 1. Copy scripts
echo "Copying scripts to $INSTALL_DIR..."
for file in "$BASE_DIR/scripts"/*.sh; do
  script_name=$(basename "$file")
  sudo cp "$file" "$INSTALL_DIR/$script_name"
  sudo chmod +x "$INSTALL_DIR/$script_name"
done

# 2. Replace placeholders and install services
echo "Installing systemd services..."
SERVICE_NAMES=("robot-ssh-init" "robot-display" "robot-eye" "robot-micro-ros-agent" "robot-interaction-controller")


for name in "${SERVICE_NAMES[@]}"; do
  input_file="$BASE_DIR/services/$name.service"
  temp_file="/tmp/$name.service"

  sed "s|%USER%|$USER_NAME|g; s|%REPO_PATH%|$REPO_PATH|g; s|%NPM_DIR%|$NPM_DIR|g" "$input_file" > "$temp_file"
  sudo cp "$temp_file" "$SYSTEMD_DIR/$name.service"
  sudo chmod 644 "$SYSTEMD_DIR/$name.service"
done

# Reload systemd to recognize new/updated service files
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

# Enable all services
for name in "${SERVICE_NAMES[@]}"; do
  sudo systemctl enable "$name.service"
done

# 3. Done
echo "Installation complete. All services will run on next boot."
echo "Repository path set to: $REPO_PATH"
echo "Scripts installed to: $INSTALL_DIR"
echo "Services enabled: ${SERVICE_NAMES[*]}"