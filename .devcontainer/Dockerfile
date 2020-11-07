# The digest (the part that starts with @sha256) isn't strictly necessary, but it ensures that the base image is always the same to prevent unnecessary rebuilds.
FROM debian:stable-slim@sha256:d22d25ae2092b04678879e3e3d9a052504982e69d68f15b7035fe7c443f34d6c

RUN apt-get update && apt-get install -y wget cmake net-tools iputils-ping git build-essential

RUN wget https://github.com/mavlink/MAVSDK/releases/download/v0.34.0/mavsdk_0.34.0_ubuntu18.04_amd64.deb \
    && dpkg -i mavsdk_0.34.0_ubuntu18.04_amd64.deb \
    && rm mavsdk_0.34.0_ubuntu18.04_amd64.deb

# OONF
RUN apt-get update && apt-get install -y libnl-3-dev
RUN wget https://github.com/OLSR/OONF/archive/v0.15.1.tar.gz \
    && tar -xzf v0.15.1.tar.gz \
    && rm v0.15.1.tar.gz \
    && cd OONF-0.15.1/build \
    && cmake .. && make \
    && make install && cd ../.. && rm -rf OONF-0.15.1

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

RUN mkdir -p ~/ros2_foxy/src
RUN cd ~/ros2_foxy \
    && wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos \
    && vcs import src < ros2.repos \
    && rosdep init \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers" --packages-skip-by-dep rviz_rendering \
    && colcon build --symlink-install
ENTRYPOINT ["sh", "-c", "source ~/ros2_foxy/install/setup.sh && \"$@\"", "-s"]

WORKDIR /workspace
