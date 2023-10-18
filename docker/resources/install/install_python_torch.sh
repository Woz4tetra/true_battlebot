#!/bin/bash

set -e

sudo chown -R 1000:1000 ${HOME}/.local

python -m pip install -r ./torch-requirements.txt

sudo -H python -m pip uninstall -y kiwisolver
sudo -H python -m pip install kiwisolver

echo "Installed python torch"
