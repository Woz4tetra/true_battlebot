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
sudo -H python -m pip install matplotlib flow_vis tqdm tensorboard hydra-core==1.3.2 mediapy loguru


sudo mkdir -p /opt/deepmind
sudo chown -R 1000:1000 /opt/deepmind

cd /opt/deepmind
git clone https://github.com/deepmind/tapnet.git
cd tapnet
sudo -H python -m pip install .
mkdir checkpoints
wget -P checkpoints https://storage.googleapis.com/dm-tapnet/causal_tapir_checkpoint.npy
