# robotiq_driver_noros

This extracts the ROS independent components from `robotiq_driver` package under
[ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
to a pure CMake package (2026-03-26: last commit on main branch is
[a41ca0e](https://github.com/PickNikRobotics/ros2_robotiq_gripper/commit/a41ca0e10cd98b067e70b76d34626d2375dde4c9)).
The package also includes the executable from `robotiq_hardware_tests` package in the repo
for testing gripper functionality.

This package only change header paths and remove ROS logging, the source files are
otherwise the same as in the original repository.

## FT 300-S sensor

`ft_sensor.{hpp,cpp}` is not from the upstream repository: it is a driver for the Robotiq
FT 300-S force torque sensor, written against the sensor manual (section 4.3) and reusing
the serial, CRC and byte utilities of the gripper driver. It reads force and torque from
the 100 Hz data stream, resynchronizing on the frame header and dropping frames with a bad
CRC, and falls back to Modbus RTU register reads (FC03) when the stream is stopped.

No member of `FTSensor` throws, so a control loop can call it directly: a lost frame, a
silent sensor, a rejected request and an unplugged converter are all return values, with
the reason in `last_error()`. `reconnect()` reopens the port and restarts the stream when
the caller has seen enough failures to conclude the connection is gone.

Two details of the sensor contradict the manual, both found on hardware: the stream only
stops for an uninterrupted burst of `0xff` bytes (50 of them sent one at a time are
ignored), and FC03 answers most significant byte first, not least as the manual's Modbus
note claims. Only the data stream is little endian.

`ft_sensor_test` exercises a connected sensor:

```
ft_sensor_test --port /dev/ttyUSB0 --baudrate 19200 --samples 100
```

`ft_sensor_test --selftest` checks the frame parsing against a scripted stream, no sensor
needed.
