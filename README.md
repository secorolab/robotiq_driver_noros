# robotiq_driver_noros

This extracts the ROS independent components from `robotiq_driver` package under
[ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
to a pure CMake package (2026-03-26: last commit on main branch is
[a41ca0e](https://github.com/PickNikRobotics/ros2_robotiq_gripper/commit/a41ca0e10cd98b067e70b76d34626d2375dde4c9)).
The package also includes the executable from `robotiq_hardware_tests` package in the repo
for testing gripper functionality.

This package only change header paths and remove ROS logging, the source files are
otherwise the same as in the original repository.
