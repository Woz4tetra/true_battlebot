#!/bin/bash

export DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))

cd ${DIR}

cat <<EOT > ${DIR}/.env
ORGANIZATION=$(${DIR}/../docker/get_organization)
IMAGE_VERSION=$(${DIR}/../docker/get_image_tag)
PERCEPTION_IMAGE_VERSION=$(${DIR}/../docker/get_image_tag perception)
PROJECT_DIR=$(realpath "${DIR}/../")
PROJECT_NAME=$(${DIR}/../docker/get_project_name)
EOT

