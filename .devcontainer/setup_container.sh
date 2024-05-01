#!/bin/bash

export DIR=$(dirname $(realpath ${BASH_SOURCE[0]}))

cd ${DIR}
PROJECT_DIR=$(realpath "${DIR}/../")
DOCKER_ROOT_DIR=${PROJECT_DIR}/docker

which nvidia-smi > /dev/null
if [[ $? -eq 1 ]]; then
	HAS_GPU="disable"
else
	nvidia-smi -L > /dev/null
	num_gpus=$?
	if ((num_gpus == 0)); then
		HAS_GPU="enable"
	else
		HAS_GPU="disable"
	fi
fi

ln -sf docker-compose.gpu-${HAS_GPU}.yaml docker-compose.gpu.yaml

cat <<EOT > ${DIR}/.env
ORGANIZATION=$(${DOCKER_ROOT_DIR}/get_organization)
IMAGE_VERSION=$(${DOCKER_ROOT_DIR}/get_image_tag)
PERCEPTION_IMAGE_VERSION=$(${DOCKER_ROOT_DIR}/get_image_tag perception)
PROJECT_DIR=${PROJECT_DIR}
PROJECT_NAME=$(${DOCKER_ROOT_DIR}/get_project_name)
EOT
