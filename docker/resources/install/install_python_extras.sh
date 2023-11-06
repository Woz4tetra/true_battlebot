#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

sudo python -m pip install --no-cache-dir -r ${BASE_DIR}/extra-requirements.txt

echo "Installed python extras"
