# wheeltec14 AKM Extended Telemetry Parser

This folder mirrors the files that should replace the current
`/home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot` parser on wheeltec14.

Copy the files into the existing catkin package, then rebuild the workspace:

```bash
cp ros/wheeltec14/turn_on_wheeltec_robot/include/wheeltec_robot.h \
  /home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot/include/wheeltec_robot.h
cp ros/wheeltec14/turn_on_wheeltec_robot/src/wheeltec_robot.cpp \
  /home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp
cp ros/wheeltec14/turn_on_wheeltec_robot/CMakeLists.txt \
  /home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot/CMakeLists.txt
cp ros/wheeltec14/turn_on_wheeltec_robot/package.xml \
  /home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot/package.xml
cp ros/wheeltec14/turn_on_wheeltec_robot/msg/*.msg \
  /home/wheeltec/wheeltec_robot/src/turn_on_wheeltec_robot/msg/
cd /home/wheeltec/wheeltec_robot
catkin_make
```

The new parser accepts the STM32 AKM 72-byte frame first. It falls back to the
original 24-byte frame, so the node can still run against older firmware during
transition. When the 72-byte frame is used, its embedded 24-byte legacy frame is
decoded through the original data path and the new lower-level fields are
published on `/wheeltec/akm_state`, `/wheeltec/control_debug`, and
`/wheeltec/chassis_diagnostics`.
