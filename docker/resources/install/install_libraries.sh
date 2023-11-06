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
make -j4
sudo make install

# tbb
cd /tmp
git clone https://github.com/wjakob/tbb.git
cd tbb
git checkout tbb44u4
cd /tmp/tbb/build
cmake ..
make -j4
sudo make install

# g2o
cd /tmp
git clone https://github.com/RainerKuemmerle/g2o.git
cd /tmp/g2o
git checkout 20201223_git
mkdir build
cd /tmp/g2o/build
cmake ..
make -j4
sudo make install

# clean up
sudo ldconfig
rm -r /tmp/* || true

echo "Built and installed all basic libraries"
