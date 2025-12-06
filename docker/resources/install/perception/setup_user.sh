#!/bin/bash

set -e

apt-get update
apt-get install -y sudo
echo DEBIAN_FRONTEND=noninteractive >> /etc/environment
groupadd -g 1001 ${USER}
useradd -r -u 1001 -m -s /bin/bash -g ${USER} -G dialout,plugdev,video,audio,sudo ${USER}
chown -R 1001:1001 ${HOME}
chown -R 1001:1001 /usr/local/
adduser ${USER} sudo
echo "${USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

echo "Setup user script complete"
