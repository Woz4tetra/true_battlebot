FROM nvidia/cuda:12.1.1-devel-ubuntu22.04

ARG PROJECT_NAME
ARG ORGANIZATION

ENV PROJECT_NAME=${PROJECT_NAME}
ENV ORGANIZATION=${ORGANIZATION}
ENV PYTHON_INSTALL_VERSION=3.11
ENV UBUNTU_VERSION_MAJOR=22
ENV UBUNTU_VERSION_MINOR=04
ENV CUDA_VERSION_MAJOR=12
ENV CUDA_VERSION_MINOR=1
ENV CUDNN_VERSION_MAJOR=8
ENV ZED_VERSION_MAJOR=4
ENV ZED_VERSION_MINOR=1

ENV DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash
SHELL ["/bin/bash", "-c"] 

# ---
# Base installation steps
# ---

WORKDIR /

# ---
# User setup
# ---

ENV USER=${ORGANIZATION}
ENV HOME=/home/${USER}

RUN mkdir -p /opt/${ORGANIZATION}/install
COPY --chown=1000:1000 ./install/setup_user.sh /opt/${ORGANIZATION}/install
RUN bash /opt/${ORGANIZATION}/install/setup_user.sh

COPY --chown=1000:1000 ./install/download /usr/bin

USER ${USER}

RUN sudo chown -R 1000:1000 /opt/${ORGANIZATION}/

# ---
# ZED SDK
# ---

ENV TZ=America/New_York

COPY --chown=1000:1000 \
    ./install/install_zed.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_zed.sh

# ---
# Basic tools
# ---

COPY --chown=1000:1000 ./install/install_basic_tools.sh /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_basic_tools.sh


# ---
# Basic dependencies
# ---

COPY --chown=1000:1000 \
    ./install/install_python_dependencies.sh \
    ./install/requirements.txt \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_python_dependencies.sh

COPY --chown=1000:1000 \
    ./install/install_apriltag.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_apriltag.sh

RUN sudo mkdir -p ${HOME}/.local && sudo chown -R 1000:1000 ${HOME}/.local

# ---
# Torch packages
# ---

COPY --chown=1000:1000 \
    ./install/perception-requirements.txt \
    ./install/install_python_perception.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_python_perception.sh

COPY --chown=1000:1000 \
    ./install/install_torchscript.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_torchscript.sh

# ---
# Python extra packages
# ---

RUN cd /usr/local/zed && python get_python_api.py

COPY --chown=1000:1000 \
    ./install/perception-extra-requirements.txt \
    ./install/install_perception_python_extras.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_perception_python_extras.sh

# ---
# Environment variables
# ---

ENV CMAKE_PREFIX_PATH=/usr/local/libtorch/share/cmake/Torch/${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}
ENV CMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/${CMAKE_LIBRARY_PATH:+:${CMAKE_LIBRARY_PATH}}

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws
ENV DEP_ROS_WS_SRC=${HOME}/dep_ws/src

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV PATH=/opt/${ORGANIZATION}/scripts${PATH:+:${PATH}} \
    PYTHONPATH=/opt/${ORGANIZATION}/${PROJECT_NAME}/perception/perception_tools:/opt/${ORGANIZATION}/${PROJECT_NAME}/shared${PYTHONPATH:+:${PYTHONPATH}} \
    PYTHONIOENCODING=utf-8 \
    PLATFORMIO_CORE_DIR=${ROS_WS_ROOT}/.platformio \
    HISTCONTROL=ignoreboth:erasedups


# ---
# launch environment
# ---

COPY --chown=1000:1000 \
    ./install/increase_file_watchers.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/increase_file_watchers.sh

COPY --chown=1000:1000 ./install/bashrc/base_bashrc ${HOME}/.bashrc
COPY --chown=1000:1000 ./install/bashrc/perception_bashrc ${HOME}/bashrc_overlay
RUN cat ${HOME}/bashrc_overlay >> ${HOME}/.bashrc && rm ${HOME}/bashrc_overlay
    
COPY --chown=1000:1000 \
    ./launch/perception.sh \
    /opt/${ORGANIZATION}/

COPY --chown=1000:1000 \
    ./install/make_perception_symlinks.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/make_perception_symlinks.sh

WORKDIR /opt/${ORGANIZATION}/${PROJECT_NAME}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
