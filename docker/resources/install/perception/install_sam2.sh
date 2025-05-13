#!/bin/bash

set -e

sudo mkdir -p /opt/facebookresearch
sudo chown -R 1000:1000 /opt/facebookresearch

cd /opt/facebookresearch
git clone https://github.com/yangchris11/samurai.git
cd samurai/sam2
sudo -H python -m pip install .
sudo -H python -m pip install ".[notebooks]"
