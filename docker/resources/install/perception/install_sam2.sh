#!/bin/bash

set -e

mkdir -p /tmp
sudo chown -R 1000:1000 /tmp

cd /tmp
git clone https://github.com/facebookresearch/sam2.git
cd sam2
sudo -H pip install -e .
