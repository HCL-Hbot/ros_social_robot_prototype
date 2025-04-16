# 🤖 Robot Startup Scripts

This project automates the startup of `Mika`, including starting a local Wi-Fi hotspot, rotating screens, enabling SSH, and launching various ROS 2 nodes like eye display and interaction controller.

Tested on **Ubuntu 24.04** with **ROS 2 Jazzy**.

---

## ✅ Features

- Fully automatic boot configuration
- Wi-Fi hotspot with environment-based configuration
- Modular installation: separate network and core setup
- Display rotation (dual HDMI supported)
- SSH auto-start on boot (via openssh-server)
- Launches ROS 2 nodes via systemd

---

## 📦 Requirements

Ensure the following are installed:

```bash
sudo apt update
sudo apt install openssh-server network-manager x11-xserver-utils
```

> ℹ️ `openssh-server` is automatically enabled after installation. To manually stop or start the SSH service:
>
> ```bash
> sudo systemctl stop ssh
> sudo systemctl start ssh
> ```

A user needs to be added to the video and audio (system) group. This way the system has rights to the video and audio after bootup (without user input).

Adding user to video and audio user group:
```bash
sudo usermod -aG audio <your-pc-username> #In my case pc user name is hcl
sudo usermod -aG video <your-pc-username> #In my case pc user name is hcl
```
A reboot or re-login is need to apply this change!

### 🖥️ Enable X11 instead of Wayland (default on Ubuntu)

Ubuntu 24.04 uses **Wayland** by default, but for full compatibility with display rotation  it's required to use **X11**:

1. Log out of your current session.
2. At the login screen, click your username.
3. Before entering your password, click the **gear icon ⚙️** in the bottom right corner.
4. Select **"Ubuntu on Xorg"** or **"GNOME on Xorg"**.
5. Log in normally — the system will remember your choice for next time.

### 🔐 Disable Auto-Login
For the startup scripts and display services to work correctly, auto-login must be disabled.
If auto-login is enabled, the system will boot to the login screen and no services or scripts will be able to interact with the displays.

---

## 📁 Folder Structure

```
robot_start_up/
├── install.sh                         # Installs all systemd services (display, micro-ROS-agent, interaction-controller launch)
├── uninstall.sh                       # Removes above services
├── install_network.sh                 # Installs only Wi-Fi hotspot service and only boots up with Wi-Fi hotspot
├── uninstall_network.sh               # Uninstalls only the hotspot service
├── scripts/
│   ├── hotspot/
│   │   ├── .env                       # Active hotspot configuration
│   │   ├── .env.example               # Example config
│   │   ├── start_hotspot.sh           # Starts the hotspot based on .env
│   │   └── stop_hotspot.sh            # Stops the hotspot
│   └── rotate_screens.sh              # Rotates HDMI displays
├── services/
│   ├── robot-display.service                # Rotates HDMI screens at boot
│   ├── robot-eye.service                    # Starts eye_display_lld (Electron)
│   ├── robot-hotspot.service                # Starts hotspot from .env
│   ├── robot-interaction-controller.service # Starts ROS interaction_controller and depending nodes (see launch file)
│   ├── robot-micro-ros-agent.service        # Starts micro-ROS Agent
└── README.md
```

---

## ⚙️ Hotspot Configuration

Copy and customize the env file:

```bash
cp scripts/hotspot/.env.example scripts/hotspot/.env
nano scripts/hotspot/.env
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
chmod +x scripts/*.sh scripts/hotspot/*.sh
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

To remove all services except the hotspot:

```bash
./uninstall.sh
```

To remove only the hotspot service:

```bash
sudo bash ./uninstall_network.sh
```

⚠️ **Warning**: Do not run `uninstall_network.sh` remotely unless you have an alternate network connection — it will disable the current hotspot.

---

## 🔧 Startup Order

The services are managed by `systemd` and are automatically started in a specific sequence. Only some of them are interdependent:

- `robot-micro-ros-agent.service` is independent and can start early.
- The following services depend on each other and must start in order:

```
robot-display.service ➝ robot-eye.service ➝ robot-interaction-controller.service
```

### Details:

1. **robot-micro-ros-agent.service** – starts the micro-ROS agent on `/dev/ttyACM0` at 115200 baud (no dependencies)
2. **robot-display.service** – rotates HDMI displays (must run before eye node)
3. **robot-eye.service** – launches the `eye_display_lld`
4. **robot-interaction-controller.service** – launches the ROS interaction_controller and dependent nodes (like camera/audio/etc.)

Dependencies are defined via After= and Requires= in each .service file

---
## 🧪 Manual Testing

You can test each service individually with:

```bash
sudo systemctl start robot-hotspot.service
sudo systemctl start robot-ssh-init.service
sudo systemctl start robot-display.service
sudo systemctl start robot-eye.service
sudo systemctl start robot-micro-ros-agent.service
sudo systemctl start robot-interaction-controller.service

journalctl -u robot-interaction-controller.service #Check logs of robot-interaction-controller.service

journalctl -u robot-interaction-controller.service -n 50 #Check the last 50 logs
```

---

## 🧪 Possible errors and their fixes
At the moment is possible that the face recogonition model (in camera\_hld) does not recognize any face. A quick fix for this is to restart this node. We van simply do this by restarting the ``robot-interaction-controller.service``:

```bash
sudo systemctl stop robot-interaction-controller.service
sudo systemctl start robot-interaction-controller.service
```

Any service can be restarted by using ``sudo systemctl stop <service_name>`` and ``sudo systemctl start <service_name>`` command 

## 🧪 Testing Individual Scripts

From the `scripts/` and `scripts/hotspot/` folders, test like so:

- Rotate screens:
  ```bash
  ./scripts/rotate_screens.sh
  ```

- Start hotspot:
  ```bash
  ./scripts/hotspot/start_hotspot.sh
  ```

- Stop hotspot:
  ```bash
  ./scripts/hotspot/stop_hotspot.sh
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

## 💥 Debugging

View logs for a service:

```bash
journalctl -u robot-hotspot.service
journalctl -u robot-interaction-controller.service
```

Check network status:

```bash
nmcli device show
nmcli connection show
nmcli connection show --active
```

Check if `dnsmasq` is running:

```bash
journalctl -u NetworkManager | grep dnsmasq
```

---


## 🧼 Notes

- Scripts are installed to `/usr/local/bin/`.
- All services are managed by systemd and will auto-start on reboot.
- The `.env` file controls Wi-Fi hotspot behavior.
- Reboot the system to apply full behavior.
- Hotspot gateway IP should be `10.42.0.1` — make sure devices can route via this address.

---

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

   ```bash
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




