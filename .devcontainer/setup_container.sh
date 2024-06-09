#!/bin/bash

export DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))

cd ${DIR}
PROJECT_DIR=$(realpath "${DIR}/../")
DOCKER_ROOT_DIR=${PROJECT_DIR}/docker

HAS_GPU=$(${DOCKER_ROOT_DIR}/scripts/has_gpu)
ln -sf docker-compose.gpu-${HAS_GPU}.yaml docker-compose.gpu.yaml

cat <<EOT > ${DIR}/.env
ORGANIZATION=$(${DOCKER_ROOT_DIR}/get_organization)
IMAGE_VERSION=$(${DOCKER_ROOT_DIR}/get_image_tag)
PERCEPTION_IMAGE_VERSION=$(${DOCKER_ROOT_DIR}/get_image_tag perception)
PROJECT_DIR=${PROJECT_DIR}
PROJECT_NAME=$(${DOCKER_ROOT_DIR}/get_project_name)
EOT
