# ros_social_robot_prototype
## Project todolist

1. Proof that we have a technical working robot with some sensors and actuators using the robot model (LLD -> HLD -> basic controller). Drivers for the following sensor en actuators should be implemented:
- [ ] Camera(s)
- [ ] Radar(s)
- [ ] Microphone
- [ ] Speaker
- [ ] Ledmatrix RGB
- [ ] LCD screen(s)
2. Define the real controller behaviour for this project with projectmananger and implement this.
3. Write about project and how to use it.


Notes:
colcon build --cmake-args -DBUILD_TESTING=OFF  -> Bouw niet de test cases / gtest

colcon build -> bouw alles inclusief gtest (onderliggend gebruikt het -DBUILD_TESTING=ON)
colcon build --cmake-args -DBUILD_TESTING=ON -> Bouwt het project ook met test cases / gtest.

Wanneer test cases zijn gebouw kan je het draaien met

colcon build test -> Hiermee wordt 


colcon build --packages-select <name-of-pkg> --cmake-args -DBUILD_TESTING=ON -DAMENT_LINT_AUTO_EXCLUDE=ON

colcon build --packages-select camera_hld --cmake-args -DBUILD_TESTING=ON -DAMENT_LINT_AUTO_EXCLUDE=ON




