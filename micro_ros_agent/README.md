# Micro-ROS Agent
This directory shows how to install and use the Micro-ROS Agent.  
The Micro-ROS Agent must be running on your machine in order to receive messages from Micro-ROS nodes (e.g. ESP32).  
It acts as a **bridge** between standard ROS 2 nodes and Micro-ROS nodes.

---

## Installation Instructions

### 1. Clone and build the micro_ros_setup package

Run the following commands **inside this directory**:

```bash
# Source the ROS 2 installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git micro_ros_setup

# Update dependencies using rosdep
sudo apt update && rosdep update
rosdep install --from-paths micro_ros_setup --ignore-src -y

# Optional: ensure pip is installed
sudo apt-get install python3-pip

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

### 2. Create and build the Micro-ROS Agent workspace
```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build step
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

## Running the Micro-ROS Agent
Before running the agent, always make sure you’ve sourced the local setup from **this folder**:

```bash
source install/local_setup.bash
```

### Example: Running over WiFi or Ethernet (UDP)
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### Example: Running over Serial
```bash
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/ttyACM0
```

### All aviable options for micro ros agent
```bash
ros2 run micro_ros_agent micro_ros_agent -h
```

## Useful Tools

To list connected USB devices:

```bash
ls /dev/tty*
```
Or
```bash
sudo dmesg | grep tty
```


