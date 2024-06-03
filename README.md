# DV ROS2

This package contains ROS2 wrappers for iniVation cameras. The package is based on the [dv-ros](https://gitlab.com/inivation/dv/dv-ros) repository and has been adapted to work with ROS2. The package is still under development and some features are not yet implemented. This project is not affiliated with iniVation AG.

<p align="center">
    <img src="images/event_camera_images.gif" width=1000>
</p>

## Installation

Tested on Ubuntu 22.04 with ROS2 Humble.
First, clone the repository.
    
```
git clone https://github.com/Telios/dv-ros2.git
```

For ease of use, build the packages with docker:
```
cd docker && docker compose up
xhost +local:docker
docker compose up
```
Otherwise, follow the instructions below.

### Dependencies
Install dependencies using the following commands:

```
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install dv-processing cmake boost-inivation libopencv-dev libeigen3-dev libcaer-dev libfmt-dev liblz4-dev libzstd-dev libssl-dev
```

### Build

Build the package using the following commands:

```
source /opt/ros/humble/setup.bash
colcon build
```

### Run

To test the camera driver, run the following commands:

```
source install/setup.bash
ros2 launch dv_ros2_visualization all.launch.py
```
A GUI should open with the 4 different camera streams.

### Repository structure

The repository contains multiple projects:

- `dv_ros2_msgs`: Basic msg and srv definitions for the ROS2 interface.
- `dv_ros2_messaging`: C++ headers required to use dv-processing in ROS2.
- `dv_ros2_capture`: Camera driver node (supports live camera data streaming and aedat4 file playback)
- `dv_ros2_accumulation`: Event stream to frame/edge accumulation
- `dv_ros2_aedat4`: Convert aedat4 files to rosbags 
- `dv_ros2_runtime_modules`: DV runtime modules for integration with ROS
- `dv_ros2_visualization`: Simple visualization of events
- `dv_ros2_tracker`: Lucas-Kanade feature trackers for event and image streams
