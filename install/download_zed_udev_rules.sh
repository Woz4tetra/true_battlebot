#!/usr/bin/env bash

set -e
download zed_installer.run https://download.stereolabs.com/zedsdk/3.5/jp44/jetsons -q
bash ./zed_installer.run --tar -x './99-slabs.rules'  > /dev/null 2>&1
sudo mv "./99-slabs.rules" "./99-zed.rules"
