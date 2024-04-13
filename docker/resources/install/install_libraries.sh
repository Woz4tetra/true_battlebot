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
