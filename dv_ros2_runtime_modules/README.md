# DV ROS2 Runtime Modules

This is a port of the original package [dv-ros-runtime-modules](https://gitlab.com/inivation/dv/dv-ros/-/tree/master/dv_ros_runtime_modules) from iniVation AG to ROS2. It provides modules for DV which publishes images / events from the DV Software into ROS2.

## Build instructions

The package was tested with ROS2 Humble on Ubuntu 22.04 LTS.
Build the package with colcon:

```bash
cd <path-to-your-workspace>
colcon build
```

1. After building the project, open DV-GUI in a terminal with the sourced ros2 workspace:

```bash
source <path-to-your-workspace>/install/setup.bash
dv-gui
```

2. The module binary `.so` files will be available in the `<path-to-your-workspace>/build/dv_ros2_runtime_modules/` directory. To load the modules, open the DV-GUI and go to `Structure -> Add module -> Modify module search paths` and select the directory path which has the files in it.

3. After selecting the path, the modules will be available in the `Add module` menu.

## Available modules

* ROS2 Event bridge - Event camera publisher, publishes events and camera calibration using DV calibration format.

* ROS2 Image bridge - Simple image frame publisher.

* ROS2 IMU bridge - Publishes IMU messages.

* ROS2 Trigger bridge - Publishes trigger messages.

