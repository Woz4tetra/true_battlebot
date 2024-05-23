#!/bin/bash

BASE_DIR=$(realpath "$(dirname $0)")
ORG_DIR=${BASE_DIR}/../..
export PROJECT_DIR=$(realpath ${ORG_DIR}/${PROJECT_NAME})
export SEARCH_DIR=${PROJECT_DIR}/ros_ws/$1

python <<EOF
import os
PROJECT_DIR = os.environ["PROJECT_DIR"]
SEARCH_DIR = os.environ["SEARCH_DIR"]
check_dirs = []
for root, dirs, files in os.walk(SEARCH_DIR):
    for dir in dirs:
        if dir in ("src", "scripts"):
            check_dirs.append(os.path.join(root, dir))
check_dirs.append(os.path.join(PROJECT_DIR, "ros_ws", "bw_tools"))
check_dirs.append(os.path.join(PROJECT_DIR, "shared", "bw_shared"))
check_dirs.append(os.path.join(PROJECT_DIR, "scripts"))

filtered_check_dirs = []
for dir in check_dirs:
    for root, dirs, files in os.walk(dir):
        found = False
        for file in files:
            if file.endswith(".py"):
                filtered_check_dirs.append(dir)
                found = True
                break
        if found:
            break
for dir in filtered_check_dirs:
    print(dir)
EOF
