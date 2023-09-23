#!/bin/bash

set -e

echo DEBIAN_FRONTEND=noninteractive >> /etc/environment
groupadd -g 1000 ${USER}
useradd -r -u 1000 -m -s /bin/bash -g ${USER} -G dialout,plugdev,video,audio,sudo ${USER}
chown -R 1000:1000 ${HOME}
chown -R 1000:1000 /usr/local/
adduser ${USER} sudo
echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

echo "Setup user script complete"
