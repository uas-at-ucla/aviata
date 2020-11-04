# The digest (the part that starts with @sha256) isn't strictly necessary, but it ensures that the base image is always the same to prevent unnecessary rebuilds.
FROM debian:stable-slim@sha256:d22d25ae2092b04678879e3e3d9a052504982e69d68f15b7035fe7c443f34d6c

RUN apt-get update && apt-get install -y wget cmake net-tools

RUN wget https://github.com/mavlink/MAVSDK/releases/download/v0.34.0/mavsdk_0.34.0_ubuntu20.04_amd64.deb \
    && dpkg -i mavsdk_0.34.0_ubuntu20.04_amd64.deb

# OONF
RUN apt-get update && apt-get install -y build-essential libnl-3-dev
RUN wget https://github.com/OLSR/OONF/archive/v0.15.1.tar.gz \
    && tar -xzf v0.15.1.tar.gz \
    && rm v0.15.1.tar.gz \
    && cd OONF-0.15.1/build \
    && cmake .. && make

WORKDIR /workspace
