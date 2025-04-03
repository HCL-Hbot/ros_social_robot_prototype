# 🤖 Robot Startup Scripts

This project automates the startup of ``Mika``, including starting a local Wi-Fi hotspot, rotating screens, enabling SSH, and launching ROS 2 display node.

Tested on **Ubuntu 24.04** with **ROS 2 Jazzy**.

---

## ✅ Features

- Fully automatic boot configuration
- Wi-Fi hotspot with environment-based configuration
- Modular installation: separate network and core setup
- Display rotation (dual HDMI supported)
- SSH auto-start on boot

---

## 📦 Requirements

Ensure the following are installed:

```bash
sudo apt update
sudo apt install openssh-server network-manager x11-xserver-utils
```

Additional requirements:

- ROS 2 workspace at `~/ros2_ws`
- ROS 2 sourced in your `eye_launch.py` or environment
- X11 display server (not Wayland)

---

## 📁 Folder Structure

```
robot_start_up/
├── install.sh                   # Installs SSH, screen rotation, eye display
├── uninstall.sh                 # Removes above services
├── install_network.sh           # Installs only Wi-Fi hotspot service
├── uninstall_network.sh         # Uninstalls only the hotspot service
├── scripts/
│   ├── .env.example             # Example env file
│   ├── rotate_screens.sh        # Rotates HDMI displays
│   ├── start_hotspot.sh         # Creates & activates hotspot from .env
│   ├── stop_hotspot.sh          # Stops the hotspot
│   ├── start_ssh.sh             # Enables SSH and starts server
│   └── stop_ssh.sh              # Disables SSH and stops server
├── services/
│   ├── robot-display.service         # Starts eye_display_lld
│   ├── robot-hotspot.service         # Starts the hotspot
│   ├── robot-launch-eye.service      # Launches ROS 2 node
│   └── robot-ssh-init.service        # Enables SSH
└── README.md
```

---

## ⚙️ Hotspot Configuration

Copy and customize the env file:

```bash
cp scripts/.env.example .env
nano .env
```

Example content:

```env
HOTSPOT_SSID=Mika
HOTSPOT_PASSWORD=changeme123
HOTSPOT_INTERFACE=wlp2s0
HOTSPOT_CONN_ID=robot-hotspot
```

---

## 🚀 Installation

Make scripts executable:

```bash
chmod +x install.sh uninstall.sh install_network.sh uninstall_network.sh
chmod +x scripts/*.sh
```

Then install the core services:

```bash
./install.sh
```

Optionally, install hotspot service:

```bash
./install_network.sh
```

---

## ❌ Uninstall

To remove everything except hotspot:

```bash
./uninstall.sh
```

To remove the hotspot service:

```bash
./uninstall_network.sh
```
⚠️ Do not run `uninstall_network.sh` remotely unless you have another connection to the robot — it will disconnect the active hotspot.

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

## 🧪 Testing Individual Scripts

You can test each script in the `scripts/` folder individually:

- Rotate screens:
  ```bash
  ./scripts/rotate_screens.sh
  ```

- Start hotspot:
  ```bash
  ./scripts/start_hotspot.sh
  ```

- Stop hotspot:
  ```bash
  ./scripts/stop_hotspot.sh
  ```

- Enable SSH:
  ```bash
  ./scripts/start_ssh.sh
  ```

- Disable SSH:
  ```bash
  ./scripts/stop_ssh.sh
  ```
---

## 🔁 Display Rotation Notes

The `rotate_screens.sh` script will:

- Detect whether X11 is running
- Rotate HDMI displays to portrait mode
- Allow manual overrides:

```bash
MAIN_ROTATION=normal SECOND_ROTATION=left ./scripts/rotate_screens.sh
```

---

## 🛠️ Debugging

View logs for the hotspot service:

```bash
journalctl -u robot-hotspot.service
```

Check current network status:

```bash
nmcli device show
nmcli connection show
nmcli connection show --active
```

Check if `dnsmasq` is running:

```bash
journalctl -u NetworkManager | grep dnsmasq
```

## 🧼 Notes

- Scripts are installed to `/usr/local/bin/`.
- All services are managed by systemd and will auto-start on reboot.
- The `.env` file controls Wi-Fi hotspot behavior.
- Reboot the system to apply full behavior.
- Hotspot gateway IP should be `10.42.0.1` — make sure devices can route via this address.


## 🔮 Future Improvements

### 📺 Runtime Display Rotation Support

Currently, the `rotate_screens.sh` script is triggered once at startup via the `robot-display.service`. This ensures screens are correctly rotated when the system boots, but does **not** handle dynamic screen changes at runtime — for example, when an HDMI display is plugged in or unplugged while the system is already running.

There are two possible improvements to enable dynamic behavior:

1. **Use udev triggers:**
   - Set up a udev rule or ACPI hook to trigger `rotate_screens.sh` whenever a new display is detected or changed.
   - This requires some experimentation and testing on specific hardware.

   Udev is ideal for detecting hardware events, like plugging in a monitor. The steps would look something like this:

   1. Create a new udev rule:

    ```bash
    sudo nano /etc/udev/rules.d/99-display-hotplug.rules
    ```
    Add the following line:
    ```
    SUBSYSTEM=="drm", ACTION=="change", RUN+="/usr/local/bin/rotate_screens.sh"
    ```
    2. Make sure the script is executable:
    ``` bash
    chmod +x /usr/local/bin/rotate_screens.sh
    ```

    3. Reload the rules:
    ```bash
    sudo udevadm control --reload-rules && sudo udevadm trigger

    ```

2. **Use systemd timer with auto-restart:**
   - Modify `robot-display.service` to include:
     ```ini
     Restart=always
     RestartSec=5
     ```
   - This will rerun the rotation script every 5 seconds, reapplying the correct orientation dynamically.
   - Note: This method may consume unnecessary resources if not rate-limited.

Until this is implemented, the screen rotation only applies during the system boot process.