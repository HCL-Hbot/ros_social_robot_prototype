#!/bin/bash

echo "Installing robot services..."

BASE_DIR="$(cd "$(dirname "$0")"; pwd)"
SCRIPT_SRC="$BASE_DIR/scripts"
SERVICE_SRC="$BASE_DIR/services"
ROBOT_USER=$(logname)

## Precondition: Check if .env file exists for wifi setup

# Setup /etc/robot_start_up/.env
ENV_TARGET="/etc/robot_start_up"

if [ -f "$SCRIPT_SRC/.env" ]; then
    sudo mkdir -p "$ENV_TARGET"
    echo "📄 Using your local .env file"
    sudo cp "$SCRIPT_SRC/.env" "$ENV_TARGET/.env"
elif [ -f "$SCRIPT_SRC/.env.example" ]; then
    sudo mkdir -p "$ENV_TARGET"
    echo "📄 No .env found — using .env.example as fallback"
    sudo cp "$SCRIPT_SRC/.env.example" "$ENV_TARGET/.env"
else
    echo "No .env or .env.example found in $SCRIPT_SRC. Cannot configure hotspot."
    exit 1
fi

##-------------------------------------------------------

# Make scripts executable
chmod +x "$SCRIPT_SRC"/*.sh

# Copy scripts to system path
echo "Copying scripts to /usr/local/bin..."
sudo cp "$SCRIPT_SRC"/*.sh /usr/local/bin/

# Replace 'user' placeholder in services with actual username
TEMP_DIR=$(mktemp -d)
for FILE in "$SERVICE_SRC"/*.service; do
  sed "s/user/$ROBOT_USER/g" "$FILE" > "$TEMP_DIR/$(basename "$FILE")"
done

# Copy systemd services
echo "Installing systemd services..."
sudo cp "$TEMP_DIR"/*.service /etc/systemd/system/
sudo systemctl daemon-reexec

# Enable services
echo "Enabling services..."
sudo systemctl enable robot-hotspot.service
sudo systemctl enable robot-ssh-init.service
sudo systemctl enable robot-display.service
#sudo systemctl enable robot-launch-eye.service

# Clean up temp
rm -r "$TEMP_DIR"

echo "Installation complete. Services will run on next boot."
