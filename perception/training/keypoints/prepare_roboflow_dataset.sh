#!/bin/bash

set -e

BASE_DIR=$(realpath "$(dirname "${0}")")

ROBOFLOW_DATASET=$1
SYNTHETIC_DATASET=$2

FLATTENED_ROBOFLOW_DATASET=${ROBOFLOW_DATASET%/}_flat
REORDERED_ROBOFLOW_DATASET=${FLATTENED_ROBOFLOW_DATASET%/}_reorder
PREPARED_DATASET=${ROBOFLOW_DATASET%/}_prepared

KEYPOINT_NAMES_CONFIG=${BASE_DIR}/keypoint_names_v2.toml

# Flatten Roboflow dataset
python ${BASE_DIR}/flatten_yolo_dataset.py ${ROBOFLOW_DATASET}

# Reorder indices
python reorder_keypoints.py ${FLATTENED_ROBOFLOW_DATASET} ${BASE_DIR}/roboflow_keypoint_names.toml ${KEYPOINT_NAMES_CONFIG}

# Combine synthetic and roboflow datasets
cp ${SYNTHETIC_DATASET}/* ${REORDERED_ROBOFLOW_DATASET}

# Split dataset into train, val, and test
python ${BASE_DIR}/split_yolo_dataset.py ${REORDERED_ROBOFLOW_DATASET} ${PREPARED_DATASET} ${KEYPOINT_NAMES_CONFIG} -t 0.9

echo Dataset: ${REORDERED_ROBOFLOW_DATASET}
