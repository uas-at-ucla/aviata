#!/bin/bash

# This script is for Raspberry Pi OS based on buster

set -e

sudo apt update -y
sudo apt upgrade -y

sudo apt install -y wget cmake net-tools iputils-ping git build-essential

# locale
sudo sed -i 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# # MAVSDK (now included as a git submodule in controls/src)
# cd ~
# (git clone https://github.com/mavlink/MAVSDK.git && sudo apt install -y colordiff doxygen && \
# cd MAVSDK && \
# git submodule update --init --recursive && \
# cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -Bbuild/default -H. -DCMAKE_CXX_STANDARD_LIBRARIES="-latomic" && \
# cmake --build build/default && \
# sudo cmake --build build/default --target install) || echo "Skipping MAVSDK"

# apriltag
cd ~
(git clone https://github.com/AprilRobotics/apriltag.git && cd apriltag && \
cmake . && sudo make install) || echo "Skipping apriltag"

# OONF (custom fork)
cd ~
(git clone https://github.com/AusarYao/OONF.git && sudo apt install -y libnl-3-dev && \
cd OONF/build && \
cmake .. && \
make) || echo "Skipping OONF"

# ROS2
cd ~
sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-setuptools \
  wget
sudo python3 -m pip install \
  colcon-common-extensions \
  rosdep \
  vcstool
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev

mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos
(sudo rosdep init && sudo rosdep fix-permissions) || echo "Skipping rosdep init"
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers python3-catkin-pkg-modules python3-lark-parser python3-ifcfg python3-rosdistro-modules"
sudo apt autoremove -y
sudo python3 -m pip install catkin-pkg lark-parser ifcfg rosdistro
sudo python3 -m pip install docutils pyyaml
colcon build --symlink-install --packages-skip rviz_rendering qt_gui_cpp turtlesim rviz_ogre_vendor --packages-skip-by-dep rviz_rendering qt_gui_cpp turtlesim rviz_ogre_vendor --cmake-args " -DCMAKE_CXX_STANDARD_LIBRARIES=-latomic" " -DCMAKE_C_STANDARD_LIBRARIES=-latomic"

echo "AVIATA Setup Finished Successfully!"
