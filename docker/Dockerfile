FROM ubuntu:22.04
SHELL ["/bin/bash", "-c"]

# make ros2 workspaces
RUN mkdir -p ./ros2_ws/src

# clone the necessary repositories for ROS2
WORKDIR /ros2_ws/src
COPY . /ros2_ws/src/

RUN apt update && apt install locales && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && export LANG=en_US.UTF-8 && apt install software-properties-common curl build-essential -y
RUN apt update && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata
RUN add-apt-repository universe -y && add-apt-repository ppa:inivation-ppa/inivation
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# build the ROS2 workspace
WORKDIR /ros2_ws
RUN apt-get update && apt-get install -y dv-processing dv-gui dv-runtime-dev ros-humble-desktop ros-dev-tools
RUN source /opt/ros/humble/setup.bash && colcon build

#  Set entrypoint
RUN mkdir -p /usr/local/bin/scripts
COPY docker/workspace-entrypoint.sh /usr/local/bin/scripts/workspace-entrypoint.sh
RUN chmod +x /usr/local/bin/scripts/workspace-entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/scripts/workspace-entrypoint.sh" ]
