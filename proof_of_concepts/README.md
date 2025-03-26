# Proof of Concepts (POCs)

This directory contains various proof-of-concept (POC) projects.  
Each POC has its own `README.md` file that explains its purpose and how to run it.

---

## Overview of Available POCs

### `proof_of_concept_micro_ros`

[`This project`](./proof_of_concept_micro_ros/) demonstrates how to create a micro-ROS node for the ESP32 using PlatformIO and the Arduino framework.  
It showcases core ROS functionality such as publishing, subscribing, and using services.  

The core logic from this project served as the foundation for the [`ld2410_node` / `ld2410_manager_node`](../micro_ros_platform_io_ws/) in the main prototype.

---

### `eye_ui_with_python`

The [`eye_ui_with_python`](./eye_ui_with_python/) project is a simple example that shows how an eye animation can be controlled using keyboard input.  
It renders one eye per available screen (up to 2 screens supported).

---

### `eye_ui_with_electron_old`

The [`eye_ui_with_electron_old`](./eye_ui_with_electron_old/) project displays two animated eyes on a single screen that follow the mouse cursor.  
Basic blinking animations are also included.

---

### `eye_ui_with_electron`

The [`eye_ui_with_electron`](./eye_ui_with_electron/) project is the final POC for eye animations. This project served as the groundwork, which I further built upon and refined in the actual product. A great part of the code base is used for the [low-level eye display driver](../ros2_ws/src/eye_display_lld/) which is software component of the actual robot.

**Key features:**
- Detects whether one or two screens are connected.
- Displays one eye per screen, or both eyes on a single screen if only one is available.
- Includes various eye animations.
- Supports mouse control and a graphical control panel.
- Provides a control panel client for testing eye behavior and animations.

---

### `eye_ui_with_electron_symposium_demo`

The [`eye_ui_with_electron_symposium_demo`](./eye_ui_with_electron_symposium_demo/) is an extension of `eye_ui_with_electron`.  
This version adds the ability to remotely launch and control the application and connected LCD-screens. 

Insights and techniques from this project were translated into automation scripts used in the final robot demo. These scripts can be found in the [`scripts`](../scripts/) directory.