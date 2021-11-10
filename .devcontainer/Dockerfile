# The digest (the part that starts with @sha256) isn't strictly necessary, but it ensures that the base image is always the same to prevent unnecessary rebuilds.
FROM debian:buster-slim@sha256:9f1ca3f1ac7b3c291363c27cab3472777e882ab5f2c8823a8932dfd4f23f4e86

RUN apt-get update && apt-get install -y wget cmake net-tools iputils-ping git build-essential

RUN wget https://github.com/mavlink/MAVSDK/releases/download/v0.34.0/mavsdk_0.34.0_ubuntu18.04_amd64.deb \
    && dpkg -i mavsdk_0.34.0_ubuntu18.04_amd64.deb \
    && rm mavsdk_0.34.0_ubuntu18.04_amd64.deb

#ROS2
RUN apt-get update && apt-get install -y locales
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
RUN python3 -m pip install -U \
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
RUN apt-get install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
RUN apt-get install --no-install-recommends -y \
  libcunit1-dev

RUN mkdir -p /usr/share/man/man1
RUN mkdir -p ~/ros2_foxy/src
RUN cd ~/ros2_foxy \
    && wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos \
    && vcs import src < ros2.repos \
    && rosdep init \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers" \
    && colcon build --symlink-install --packages-skip rviz_rendering qt_gui_cpp turtlesim --packages-skip-by-dep rviz_rendering qt_gui_cpp turtlesim
RUN echo 'source ~/ros2_foxy/install/setup.bash; export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"."' >> ~/.bashrc

# OONF
RUN apt-get update && apt-get install -y libnl-3-dev
RUN git clone https://github.com/AusarYao/OONF.git \
    && cd OONF/build \
    && cmake .. \
    && make

# apriltag
RUN git clone https://github.com/AprilRobotics/apriltag.git \
    && cd apriltag && cmake . \
    && make install

# Newer version of MAVSDK (not needed for now)
# RUN wget https://github.com/mavlink/MAVSDK/releases/download/v0.44.0/mavsdk_0.44.0_ubuntu18.04_amd64.deb \
#     && dpkg -i mavsdk_0.44.0_ubuntu18.04_amd64.deb \
#     && rm mavsdk_0.44.0_ubuntu18.04_amd64.deb

# pigpio
RUN apt install unzip -y 
RUN wget https://github.com/joan2937/pigpio/archive/master.zip \
    && unzip master.zip \
    && cd pigpio-master \
    && make \
    && sudo make install 

