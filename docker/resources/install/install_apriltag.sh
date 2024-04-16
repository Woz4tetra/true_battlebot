#!/bin/bash

set -e

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp

cd /tmp
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
git checkout 1110fd6e81aec72e759acddc4aa5461965450597
mkdir build
cd /tmp/apriltag/build
cmake ..
make -j$(nproc)
sudo make install

# clean up
sudo ldconfig
rm -r /tmp/* || true

echo "Built and installed apriltag libraries"
