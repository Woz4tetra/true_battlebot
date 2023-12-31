#!/usr/bin/env bash

# This script should be run on the host, not in the container

# This script will setup USB rules to open the ZED cameras without root access
# This can also be useful to access the cameras from a docker container without root (this script needs to be run on the host)
# NB: Running the ZED SDK installer will already setup those

set -e
download zed_installer.run https://download.stereolabs.com/zedsdk/3.5/jp44/jetsons -q
bash ./zed_installer.run --tar -x './99-slabs.rules'  > /dev/null 2>&1
sudo mv "./99-slabs.rules" "/etc/udev/rules.d/99-zed.rules"
sudo chmod 777 "/etc/udev/rules.d/99-zed.rules"
sudo udevadm control --reload-rules && sudo udevadm trigger
