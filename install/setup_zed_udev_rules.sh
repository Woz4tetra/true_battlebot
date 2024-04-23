#!/usr/bin/env bash

# This script should be run on the host, not in the container

# This script will setup USB rules to open the ZED cameras without root access
# This can also be useful to access the cameras from a docker container without root (this script needs to be run on the host)
# NB: Running the ZED SDK installer will already setup those

set -e
sudo mv "./99-zed.rules" "/etc/udev/rules.d/99-zed.rules"
sudo chmod 777 "/etc/udev/rules.d/99-zed.rules"
sudo udevadm control --reload-rules && sudo udevadm trigger
