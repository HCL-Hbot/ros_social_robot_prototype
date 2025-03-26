# ROS2 workspace

```bash
cd ros2_ws
```

Add sourcing to your shell startup script.
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

## Table of Content

1. [How to build project without tests](#how-to-build-project-without-tests-fast-build)
2. [How to build project with testing](#how-to-build-project-with-testing)
3. [How to build one package specific](#how-to-build-one-package-specific)
4. [Run tests](#run-tests)
5. [Run test for one package](#run-test-for-one-package)

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

```bash
colcon build --packages-select camera_hdl #build one package
colcon build --event-handlers console_direct+ --packages-select camera_hld #build one package with verbose output
colcon build --packages-select eye_display_lld --cmake-clean-cache --event-handlers console_direct+ #Build with verbose output but also remove cmake cache before the build
```

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