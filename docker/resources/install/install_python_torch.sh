#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo chown -R 1000:1000 ${HOME}/.local

sudo -H python -m pip install -r ${BASE_DIR}/torch-requirements.txt

sudo -H python -m pip uninstall -y kiwisolver
sudo -H python -m pip install kiwisolver

cd /tmp
git clone https://github.com/facebookresearch/detectron2.git
cd detectron2
git checkout 5de5319a49e7ea819586d3d6a817632392dfdeb2
cd ..
python -m pip install -e detectron2
rm -rf /tmp/detectron2
cd /

echo "Installed python torch"
