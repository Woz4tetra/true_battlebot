ARG UBUNTU_VERSION_MAJOR
ARG UBUNTU_VERSION_MINOR
ARG CUDA_VERSION_MAJOR
ARG CUDA_VERSION_MINOR
ARG CUDNN_VERSION_MAJOR

FROM nvidia/cuda:${CUDA_VERSION_MAJOR}.${CUDA_VERSION_MINOR}-cudnn${CUDNN_VERSION_MAJOR}-devel-ubuntu${UBUNTU_VERSION_MAJOR}.${UBUNTU_VERSION_MINOR}

ARG ROS_DISTRO
ARG PROJECT_NAME
ARG ORGANIZATION
ARG PYTHON_INSTALL_VERSION
ARG UBUNTU_VERSION_MAJOR
ARG UBUNTU_VERSION_MINOR
ARG CUDA_VERSION_MAJOR
ARG CUDA_VERSION_MINOR
ARG CUDNN_VERSION_MAJOR

ENV ROS_DISTRO=${ROS_DISTRO}
ENV PROJECT_NAME=${PROJECT_NAME}
ENV ORGANIZATION=${ORGANIZATION}
ENV PYTHON_INSTALL_VERSION=${PYTHON_INSTALL_VERSION}
ENV CUDA_VERSION_MAJOR=${CUDA_VERSION_MAJOR}
ENV CUDA_VERSION_MINOR=${CUDA_VERSION_MINOR}
ENV UBUNTU_VERSION_MAJOR=${UBUNTU_VERSION_MAJOR}
ENV UBUNTU_VERSION_MINOR=${UBUNTU_VERSION_MINOR}

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
    ./install/install_libraries.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_libraries.sh

RUN sudo chown -R 1000:1000 ${HOME}/.local

# ---
# Torch packages
# ---

COPY --chown=1000:1000 \
    ./install/torch-requirements.txt \
    ./install/install_python_torch.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_python_torch.sh

COPY --chown=1000:1000 \
    ./install/install_torchscript.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_torchscript.sh

# ---
# ROS base workspace
# ---

ENV PATH=${HOME}/.local/bin${PATH:+:${PATH}}

ENV BASE_ROS_WS_ROOT=${HOME}/base_ws

COPY --chown=1000:1000 \
    ./install/${ROS_DISTRO}.rosinstall \
    ./install/install_ros.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_ros.sh


# ---
# ROS dependency workspace
# ---

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws

COPY --chown=1000:1000 \
    ./install/install_dep.sh \
    ./install/patches/geometry2.patch \
    ./install/patches/image-pipeline.patch \
    ./install/${PROJECT_NAME}.rosinstall \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_dep.sh /opt/${ORGANIZATION}/install/${PROJECT_NAME}.rosinstall

# ---
# Python extra packages
# ---

COPY --chown=1000:1000 \
    ./install/extra-requirements.txt \
    ./install/install_python_extras.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_python_extras.sh

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
    PYTHONPATH=/opt/${ORGANIZATION}/${PROJECT_NAME}/src/bw_tools${PYTHONPATH:+:${PYTHONPATH}} \
    PYTHONIOENCODING=utf-8 \
    PLATFORMIO_CORE_DIR=${ROS_WS_ROOT}/.platformio \
    HISTCONTROL=ignoreboth:erasedups


# ---
# launch environment
# ---

COPY --chown=1000:1000 ./install/bashrc ${HOME}/.bashrc

COPY --chown=1000:1000 \
    ./launch/set_log_format.sh \
    ./launch/launch.sh \
    ./launch/roscore.sh \
    ./launch/rosconsole.config \
    /opt/${ORGANIZATION}/

COPY --chown=1000:1000 \
    ./install/make_symlinks.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/make_symlinks.sh

RUN mkdir -p ${HOME}/.ros && \
    chown -R 1000:1000 ${HOME}/.ros

WORKDIR ${HOME}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
