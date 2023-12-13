#!/bin/bash

set -e

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp

# apriltag
cd /tmp
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
git checkout 3e8e974d0d8d6ab318abf56d87506d15d7f2cc35
mkdir build
cd /tmp/apriltag/build
cmake ..
make -j$(nproc)
sudo make install

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

# platformio
cd /tmp
download get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
export LC_ALL=en_US.utf-8
export LANG=en_US.utf-8
python get-platformio.py
sudo ln -s $HOME/.platformio/penv/bin/platformio /usr/local/bin

# clean up
sudo ldconfig
rm -r /tmp/* || true

echo "Built and installed all basic libraries"
