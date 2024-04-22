#!/bin/bash

set -e

sudo apt-get update
sudo apt-get install -y \
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
    libbullet-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libpcl-dev  \
    libnetpbm10-dev  \
    libtheora-dev  \
    libgeographic-dev  \
    libavutil-dev  \
    libswscale-dev  \
    liburdfdom-headers-dev  \
    liburdfdom-dev  \
    libsuitesparse-dev \
    python3-termcolor \
    libceres-dev \
    libgeos-dev \
    python3-catkin

sudo -H python -m pip install -U rosdep rosinstall_generator vcstool

# tbb
cd /tmp
git clone https://github.com/wjakob/tbb.git
cd tbb
git checkout tbb44u4
cd /tmp/tbb/build
cmake ..
make -j$(nproc)
sudo make install

# g2o
cd /tmp
git clone https://github.com/RainerKuemmerle/g2o.git
cd /tmp/g2o
git checkout 20201223_git
mkdir build
cd /tmp/g2o/build
cmake ..
make -j$(nproc)
sudo make install

# orocos_kinematics_dynamics
cd /tmp
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics
git checkout 7478f96d01963db105e214ba79bd43709b0a3e5a
git submodule update --init
cd orocos_kdl
mkdir build
cd build
cmake ..
make -j$(nproc)
sudo make install

# python_orocos_kdl
cd /tmp/orocos_kinematics_dynamics
mkdir -p python_orocos_kdl/build
cd python_orocos_kdl/build
cmake -D PYTHON_EXECUTABLE=/usr/bin/python \
    -D PYTHON_INCLUDE_DIR=/usr/include/python${PYTHON_INSTALL_VERSION} \
    -D PYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython${PYTHON_INSTALL_VERSION}.so \
    -D PYBIND11_PYTHON_VERSION=3 ..
make -j$(nproc)
sudo make install

# clean up
sudo ldconfig
rm -r /tmp/* || true

echo "Installed ROS deps" 
