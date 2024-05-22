# DV ROS2

This package contains ROS2 wrappers for iniVation cameras. The package is based on the [dv-ros](https://gitlab.com/inivation/dv/dv-ros) repository and has been adapted to work with ROS2.

## Installation

Tested on Ubuntu 22.04 with ROS2 Humble.
First, clone the repository.
    
```
git clone --recurse-submodules https://github.com/Telios/dv-ros2.git
```

### Dependencies
Install dependencies using the following commands:

```
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt-get update
sudo apt-get install cmake boost-inivation libopencv-dev libeigen3-dev libcaer-dev libfmt-dev liblz4-dev libzstd-dev libssl-dev
```

### Build

Build the package using the following commands:

```
source /opt/ros/humble/setup.bash
colcon build
```