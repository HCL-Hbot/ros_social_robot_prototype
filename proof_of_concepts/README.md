# Proof of Concepts (POCs)

In this directory some various POCs can be found. Every POC has it's own README file where it explains what its purpose is and how it can run.

## Brief description of each POC

### mico_ros_pub_sub 
*TODO: This project is already done but it's content and it's documentation needs to me moved to this directory. Will be done soon...*

This project shows how a micro-ros node can be created for the esp32 with platformio and arduino. Some core functionality of ROS are shown in this project.

### eye_ui_with_python
The [eye_ui_with_python](./eye_ui_with_python/) is a example that shows how a eye can be controlled with keyboard input. One eye will be drawn for each available screen (max 2).

### eye_ui_with_electron_old
The [eye_ui_with_electron_old](./eye_ui_with_electron_old/) is a example where two eyes (on one screen) follow mouse movement. Also a blinking animation is implemented. 

### eye_ui_with_electron
The [eye_ui_with_electron](./eye_ui_with_electron/) project represents the final POC for eye animation. I developed this codebase and used it as the foundation for implementing the low-level eye driver in the social robot prototype. This project served as the groundwork, which I further built upon and refined in the actual product. 

Core functionality: 
- Detect whether there is one or two screens.
- Display one eye on each screen; if there is only one screen, display both eyes on it.
- Support for various animations.
- Eye control using a mouse and a control panel.
- A control panel client to test eye functionality and animations.
