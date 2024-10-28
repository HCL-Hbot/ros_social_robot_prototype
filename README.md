# ros_social_robot_prototype
## Project todolist

1. Proof that we have a technical working robot with some sensors and actuators using the robot model (LLD -> HLD -> basic controller). Drivers for the following sensor en actuators should be implemented:
- [ ] Camera(s)
    - [x] LLD
    - [x] HLD (Mocked version)
    - [ ] Real HLD (Later in project)
- [ ] Radar(s)
- [ ] Microphone
- [ ] Speaker
- [ ] Ledmatrix RGB
- [ ] LCD screen(s)
2. Define the real controller behaviour for this project with projectmananger and implement this.
3. Write about project and how to use it.


# GENERAL
Building this project should be done in the ros2_ws directory

```bash
cd ros2_ws
```


# How to build project without tests (Fast build)
```bash
colcon build --cmake-args -DBUILD_TESTING=OFF
```

# How to build project with testing

```bash
 colcon build
```

OR

```bash
colcon build --cmake-args -DBUILD_TESTING=ON
```
## How to build one package specific
An example with camera_hld package
```bash
 colcon test --packages-select camera_hld
```
Building the project this way will include unittest and cppcheck.

## Run tests

step 1 run tests:
```bash
  colcon test
```
step 2 see results (path to xml files will be given): 
```bash
  colcon test-result --all
```

## Run test for one package

An example with the package camera_hld

step 1 run test:
```bash
  colcon test --packages-select camera_hld
```
step 2 see results (path to xml files will be given):
```bash
  colcon test-result --all
```

# Configuration for serial port access (needed for flashing to MCU)

Most likely by default your system does not have read and write acces to the serial port. In the following steps we will obtain these rights. 

There are multiple ways to gain read and write access to the serial port, we will use the most "simple" way.

1. Find your (desired) serial port. This is most likely the one that is currently connected. We can find this with:
```bash
  sudo dmesg | grep tty
```
In my case the microcontroller was connectect to ttyUSB0

2. Identify which group owns the file corresponding to the serial port communication:
```bash
  ls -l /dev/ttyUSB0
  
  # prints something like:
  # crw-rw---- 1 root dialout 188, 0 Oct 28 08:54 /dev/ttyUSB0
```

3. Add your self to the group corresponding to the serial port communication. In my case the group name was "dialout.Typical names are “dialout”, “plugdev” (Debian/Ubuntu, Fedora), or “uucp” (Arch Linux). Adding a user to a group is done by:
```bash
  sudo usermod -a -G dialout $USERNAME
```

**NOTE** **&#9432;**
You will need to log out and log back in again (or reboot) for the user group changes to take effect.


# Platform IO setup for ubuntu

1. sudo apt install -y git cmake python3-pip
2. sudo apt install python3-venv
3. install the platform io extension: search this in the extension bar in VSstudio code (ctrl+P) -> "ext install platformio.platformio-ide"    

**NOTE** **&#9432;**
To be able to flash to a MCU from platform IO, we need read and write access to the serial port. 
Follow the steps from the "Configuration for serial port access" section to gain these rights. 

Alternatively install [udev][1] rules for PlatformIO supported boards/devices. Adding udev rules for platformIO can be found [here][2] in the section "99-platformio-udev.rules". 

[1]: https://en.wikipedia.org/wiki/Udev            "udev"
[2]: https://docs.platformio.org/en/latest/core/installation/udev-rules.html           "here"

# Arduino IDE setup for ubuntu

1. Download the latest release (AppImage)

2. Find the AppImage file in your file manager.

3. Make the AppImage file executable:
  Right-click the file.
  Choose Properties,
  Select the Permissions.
  Tick the Allow executing file as program box.
  Double-click the AppImage file to launch Arduino IDE.

In case you cannot run the AppImage file, make sure that FUSE is installed on your system.

```bash
sudo add-apt-repository universe
sudo apt install libfuse2
```

**NOTE** **&#9432;**
To be able to flash to a MCU from Arduino IDE, we need read and write access to the serial port. 
Follow the steps from the "Configuration for serial port access" section to gain these rights. 

Alternatively add [udev][1] rule for Arduino IDE. To enable the Arduino IDE to access the serial port and upload code to your board, the following rule can be added to /etc/udev/rules.d/99-arduino.rules.

```bash
  SUBSYSTEMS=="usb", ATTRS{idVendor}=="2341", GROUP="“dialout”", MODE="0666"
```
