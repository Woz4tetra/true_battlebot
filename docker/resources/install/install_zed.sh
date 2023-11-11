#!/bin/bash

set -e

sudo apt-get update || true
sudo apt-get install -y --no-install-recommends apt-utils dialog

sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime
echo $TZ | sudo tee /etc/timezone 
sudo apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libopencv-dev libpq-dev zstd usbutils

sudo mkdir -p /tmp
sudo sudo chown -R 1000:1000 /tmp

ZED_SDK_URL="https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.0/ZED_SDK_Ubuntu${UBUNTU_VERSION_MAJOR}_cuda${CUDA_VERSION_MAJOR}_v4.0.8.zstd.run"
echo "Using ZED SDK URL: ${ZED_SDK_URL}"
cd /tmp
wget -q -O ZED_SDK_Linux_Ubuntu.run ${ZED_SDK_URL}
chmod +x ZED_SDK_Linux_Ubuntu.run 
sudo ./ZED_SDK_Linux_Ubuntu.run -- silent skip_python skip_cuda
sudo chown -R 1000:1000 /usr/local/zed/
sudo ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so 
sudo rm ZED_SDK_Linux_Ubuntu.run 

