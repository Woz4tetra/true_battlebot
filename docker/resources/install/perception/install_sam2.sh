#!/bin/bash

set -e

sudo mkdir -p /opt/facebookresearch
sudo chown -R 1000:1000 /opt/facebookresearch

cd /opt/facebookresearch
git clone https://github.com/facebookresearch/sam2.git
cd sam2
sudo -H python -m pip install .
sudo -H python -m pip install ".[notebooks]"

cd /opt/facebookresearch
git clone https://github.com/facebookresearch/co-tracker.git
cd co-tracker
sudo -H python -m pip install .
sudo -H python -m pip install matplotlib flow_vis tqdm tensorboard hydra-core==1.1.0 mediapy loguru
