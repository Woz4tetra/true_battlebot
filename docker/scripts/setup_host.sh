#!/bin/bash
BASE_DIR=$(realpath "$(dirname $0)")

sudo -v -p "Please enter the password for ${USER} host setup can do its thing: "

sudo ${BASE_DIR}/install_docker_dependencies ${USER}
sudo ~/true_battlebot/docker/scripts/install_nvidia_container_toolkit

mkdir -p /media/storage/true-battlebot-media/logs
sudo chown -R 1000:1000 /media/storage/true-battlebot-media

sudo cp ${BASE_DIR}/99-zed.rules /etc/udev/rules.d