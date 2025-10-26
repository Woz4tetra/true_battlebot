#!/bin/bash

set -e

sudo mkdir -p /opt/deepmind
sudo chown -R 1000:1000 /opt/deepmind

cd /opt/deepmind
git clone https://github.com/deepmind/tapnet.git
cd tapnet
sudo -H python -m pip install .
mkdir checkpoints
wget -P checkpoints https://storage.googleapis.com/dm-tapnet/causal_tapir_checkpoint.npy
wget -P checkpoints https://storage.googleapis.com/dm-tapnet/tapir_checkpoint_panning.npy
wget -P checkpoints https://storage.googleapis.com/dm-tapnet/bootstap/bootstapir_checkpoint_v2.npy
sudo python -m pip install tensorflow_datasets
sudo python -m pip install jax[cuda12]
