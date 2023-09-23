#!/bin/bash

set -e

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp

# platformio
cd /tmp
download get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python get-platformio.py
sudo ln -s $HOME/.platformio/penv/bin/platformio /usr/local/bin

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

# foxglove
cd /tmp
sudo apt-get -y install xdg-utils libnotify4
download foxglove-studio.deb https://github.com/foxglove/studio/releases/download/v1.71.0/foxglove-studio-1.71.0-linux-amd64.deb
sudo dpkg -i foxglove-studio.deb
sudo apt-get -y install foxglove-studio

# nlopt
cd /tmp
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make -j4
sudo make install

# clean up
sudo ldconfig
rm -r /tmp/* || true

echo "Built and installed all basic libraries"
