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
