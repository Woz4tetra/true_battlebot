#!/bin/bash

set -e

apt-get update || true
apt-get install -y --no-install-recommends apt-utils dialog

ln -snf /usr/share/zoneinfo/$TZ /etc/localtime
echo $TZ > /etc/timezone 
apt-get install --yes lsb-release wget less udev sudo build-essential cmake python3 python3-dev python3-pip python3-wheel git jq libopencv-dev libpq-dev zstd usbutils

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp

cd /tmp
wget -q -O ZED_SDK_Linux_Ubuntu.run  https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/4.0/ZED_SDK_Ubuntu20_cuda11.8_v4.0.7.zstd.run
chmod +x ZED_SDK_Linux_Ubuntu.run 
./ZED_SDK_Linux_Ubuntu.run -- silent skip_python skip_cuda
chown -R 1000:1000 /usr/local/zed/
ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so 
rm ZED_SDK_Linux_Ubuntu.run 

