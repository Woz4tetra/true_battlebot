#!/bin/bash
set -e
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y \
    apt-utils git nano tmux curl wget htop net-tools iproute2 iputils-ping \
    gdb dumb-init rsync entr build-essential libtool autoconf zip unzip zstd software-properties-common

sudo apt-get install -y \
    cmake \
    libboost-thread-dev \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-regex-dev \
    libboost-program-options-dev \
    libconsole-bridge-dev \
    libpoco-dev \
    libtinyxml2-dev \
    liblz4-dev \
    libbz2-dev \
    uuid-dev \
    liblog4cxx-dev \
    libgpgme-dev \
    libgtest-dev \
    python3 \
    python3-pip \
    python3-setuptools \
    libbullet-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libpcl-dev  \
    libnetpbm10-dev  \
    libtheora-dev  \
    libgeographic-dev  \
    libavutil-dev  \
    libswscale-dev  \
    v4l-utils  \
    liburdfdom-headers-dev  \
    liburdfdom-dev  \
    libusb-1.0-0*  \
    libsuitesparse-dev \
    python3-termcolor \
    libceres-dev \
    libturbojpeg-dev \
    libv4l-dev \
    libgeos-dev \
    ffmpeg
