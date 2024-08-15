#!/bin/bash

BASE_DIR=$(realpath "$(dirname $0)")

ORG_DIR=${BASE_DIR}/../..
PROJECT_DIR=$(realpath ${ORG_DIR}/${PROJECT_NAME})

echo ${PROJECT_DIR}
