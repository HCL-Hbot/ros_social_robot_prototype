#!/bin/bash

# Check if the session is running on Xorg
if [[ "$(loginctl show-session $(awk '/tty/{print $1}' <(loginctl)) -p Type --value)" != "x11" ]]; then
    echo "This script only works with X11 (Xorg). Wayland is not supported."
    exit 1
fi

# Wait for Xorg to be fully initialized
sleep 5

# Set the DISPLAY variable if not already set
if [ -z "$DISPLAY" ]; then
    export DISPLAY=:0
fi

# Check if xrandr is installed
if ! command -v xrandr &> /dev/null; then
    echo "xrandr is not installed. Please install it first."
    exit 1
fi

# Check if xrandr can access the display
if ! xrandr --query &> /dev/null; then
    echo "xrandr could not access display $DISPLAY. Is X running?"
    exit 1
fi

# Default rotation settings (if variables are not provided)
MAIN_ROTATION="${MAIN_ROTATION:-left}"
SECOND_ROTATION="${SECOND_ROTATION:-right}"

# Detect connected screens
connected_screens=$(xrandr --query | grep " connected" | awk '{ print $1 }')

# Find HDMI screens
hdmi_screens=()
for screen in $connected_screens; do
    if [[ $screen == HDMI* ]]; then
        hdmi_screens+=("$screen")
    fi
done

# If no HDMI screens are connected, exit
if [ ${#hdmi_screens[@]} -eq 0 ]; then
    echo "No HDMI screens detected."
    exit 1
fi

# Configure screens
if [ ${#hdmi_screens[@]} -eq 1 ]; then
    # If only one HDMI screen is connected, rotate it according to MAIN_ROTATION and set as primary
    xrandr --output "${hdmi_screens[0]}" --rotate "$MAIN_ROTATION" --primary 2>&1 || echo "Error configuring ${hdmi_screens[0]}"
    echo "Configured single screen (${hdmi_screens[0]}) as primary with $MAIN_ROTATION rotation."
else
    # If two HDMI screens are connected, use separate rotations
    xrandr --output "${hdmi_screens[0]}" --rotate "$MAIN_ROTATION" --primary 2>&1 || echo "Error configuring ${hdmi_screens[0]}"
    xrandr --output "${hdmi_screens[1]}" --rotate "$SECOND_ROTATION" 2>&1 || echo "Error configuring ${hdmi_screens[1]}"
    echo "Configured dual screens: ${hdmi_screens[0]} ($MAIN_ROTATION, primary) and ${hdmi_screens[1]} ($SECOND_ROTATION)."
fi