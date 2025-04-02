# 🤖 Robot Startup Scripts

This directory contains all necessary scripts and systemd services to configure your robot to:

- Start a local Wi-Fi hotspot
- Enable SSH server
- Automatically rotate screens
- Launch your ROS 2 node (`eye_display_lld`) at boot

Tested on **Ubuntu 24.04** with **ROS 2 Jazzy**.

---

## ✅ Features

- Fully automatic boot configuration
- Dynamic user detection (no hardcoded usernames)
- Environment-based configuration for hotspot
- Modular and easy to extend
- Wayland/X11 check built-in for display rotation

---

## 📦 Requirements

Make sure the following packages are installed:

```bash
sudo apt update
sudo apt install openssh-server network-manager x11-xserver-utils
```

Your setup must also include:

- A fully built ROS 2 workspace at `~/ros2_ws`
- ROS 2 sourced inside `eye_launch.py` or via environment setup
- X11 (Xorg) session (not Wayland)
- Optional: Two HDMI displays

---

## 📁 Folder Structure

```
scripts/robot_start_up/
├── install.sh
├── uninstall.sh
├── README.md
├── scripts/
│   ├── .env.example
│   ├── start_hotspot.sh
│   ├── start_ssh.sh
│   ├── rotate_screens.sh
└── services/
    ├── robot-hotspot.service
    ├── robot-ssh-init.service
    ├── robot-display.service
    └── robot-launch-eye.service
```

---

## ⚙️ Hotspot Configuration

To configure your robot's hotspot settings, edit the `.env` file:

```bash
sudo nano /etc/robot_start_up/.env
```

If no `.env` file is provided in the repo, the `install.sh` will fall back to `.env.example`.

Use these environment variables:

```env
HOTSPOT_SSID=MyRobotAP
HOTSPOT_PASSWORD=changeme123
HOTSPOT_INTERFACE=wlan0
HOTSPOT_CONN_ID=robot-hotspot
```

💡 You may include a custom `.env` file in `scripts/robot_start_up/scripts/` before installation — this will be copied into the system during install.

---

## 🚀 Installation

From the root of your repo:

```bash
cd scripts/robot_start_up
chmod +x install.sh uninstall.sh
./install.sh
```

This will:
- Copy all scripts to `/usr/local/bin/`
- Copy `.env` or `.env.example` to `/etc/robot_start_up/.env`
- Replace `user` placeholders with your current Linux username
- Register and enable systemd services for auto-start on boot

---

## ❌ Uninstall

```bash
./uninstall.sh
```

This will:
- Stop and remove all related systemd services
- Prompt to remove the `.env` file and config folder

---

## 🧪 Manual Testing

```bash
sudo systemctl start robot-hotspot.service
sudo systemctl start robot-ssh-init.service
sudo systemctl start robot-display.service
sudo systemctl start robot-launch-eye.service

journalctl -u robot-launch-eye.service
```

---

## 🔁 Display Rotation Notes

The script `rotate_screens.sh`:
- Checks if X11 is running
- Detects HDMI displays
- Applies screen rotation
- Supports rotation override via:

```bash
MAIN_ROTATION=normal SECOND_ROTATION=left ./rotate_screens.sh
```

---

## 🧰 Want to Add More Services?

1. Add a new `.service` file in `services/`
2. Write a matching script in `scripts/`
3. Use `user` as a placeholder for usernames
4. Run `./install.sh`

---