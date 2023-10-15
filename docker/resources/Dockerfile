ARG ROS_DISTRO
ARG PROJECT_NAME
ARG ORGANIZATION

FROM ros:${ROS_DISTRO}-ros-core as base_image

RUN apt-get update && \
    apt-get install -y rsync
COPY --chown=1000:1000 \
    ./install/copy_select_from_base.sh \
    ./install/include_from_base.txt \
    /tmp/
RUN bash /tmp/copy_select_from_base.sh

FROM stereolabs/zed:4.0-devel-cuda11.8-ubuntu20.04

ARG ROS_DISTRO
ARG PROJECT_NAME
ARG ORGANIZATION

ENV ROS_DISTRO=${ROS_DISTRO}
ENV PROJECT_NAME=${PROJECT_NAME}
ENV ORGANIZATION=${ORGANIZATION}
ENV PYTHON_INSTALL_VERSION=3.9

ENV DEBIAN_FRONTEND=noninteractive \
    SHELL=/bin/bash
SHELL ["/bin/bash", "-c"] 


# ---
# Base installation steps
# ---

# ---
# User setup
# ---

ENV USER=${ORGANIZATION}
ENV HOME=/home/${USER}

RUN sudo mkdir -p /opt/${ORGANIZATION}/install && sudo chown -R 1000:1000 /opt/${ORGANIZATION}/
COPY --chown=1000:1000 ./install/setup_user.sh /opt/${ORGANIZATION}/install
RUN bash /opt/${ORGANIZATION}/install/setup_user.sh

COPY --chown=1000:1000 ./install/download /usr/bin

RUN chown -R 1000:1000 /usr/local/zed/

USER ${USER}

# ---
# Basic tools
# ---

COPY --chown=1000:1000 ./install/install_basic_tools.sh /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_basic_tools.sh

# ---
# Multistage copy
# ---

COPY --from=base_image /tmp/multistage_copy /tmp/multistage_copy
COPY --chown=1000:1000 ./install/copy_over_multistage.sh /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/copy_over_multistage.sh

# ---
# PyTorch CMake
# ---

ENV LD_LIBRARY_PATH=/usr/local/lib/python3.6/dist-packages/torch/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}} \
    CMAKE_PREFIX_PATH=/usr/local/lib/python3.6/dist-packages/torch/share/cmake/Torch${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}


# ---
# Basic dependencies
# ---

COPY --chown=1000:1000 \
    ./install/install_apt_packages.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_apt_packages.sh
COPY --chown=1000:1000 \
    ./install/install_python_dependencies.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_python_dependencies.sh
COPY --chown=1000:1000 \
    ./install/install_libraries.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_libraries.sh

# ---
# ROS dependency workspace
# ---

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws \
    DEP_ROS_WS_SRC=${HOME}/dep_ws/src

COPY --chown=1000:1000 ./install/rosdep_install.sh /opt/${ORGANIZATION}/install
COPY --chown=1000:1000 \
    ./install/install_ros_packages.sh \
    ./install/${PROJECT_NAME}.rosinstall \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_ros_packages.sh /opt/${ORGANIZATION}/install/${PROJECT_NAME}.rosinstall y

# ---
# Python extra packages
# ---

COPY --chown=1000:1000 \
    ./install/requirements.txt \
    ./install/install_python_extras.sh \
    /opt/${ORGANIZATION}/install/
RUN cd /opt/${ORGANIZATION}/install && bash ./install_python_extras.sh

# ---
# GPU image dependencies
# ---

COPY --chown=1000:1000 \
    ./install/install_torchscript.sh \
    /opt/${ORGANIZATION}/install/
RUN bash -x /opt/${ORGANIZATION}/install/install_torchscript.sh

# ---
# Environment variables
# ---

ENV CMAKE_PREFIX_PATH=/usr/local/libtorch/share/cmake/Torch/${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}
ENV CMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs/${CMAKE_LIBRARY_PATH:+:${CMAKE_LIBRARY_PATH}}

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws
ENV DEP_ROS_WS_SRC=${HOME}/dep_ws/src

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV FLASK_ENV=development \
    PATH=${HOME}/.local/bin:/opt/${ORGANIZATION}/scripts${PATH:+:${PATH}} \
    PYTHONPATH=/opt/${ORGANIZATION}/${PROJECT_NAME}/src/bw_tools${PYTHONPATH:+:${PYTHONPATH}} \
    MYPYPATH=/opt/${ORGANIZATION}/${PROJECT_NAME}/src/bw_tools${MYPYPATH:+:${MYPYPATH}} \
    PYTHONIOENCODING=utf-8

# ---
# launch environment
# ---

COPY --chown=1000:1000 ./install/bashrc ${HOME}/.bashrc

COPY --chown=1000:1000 \
    ./launch/entrypoint.sh \
    ./launch/launch.sh \
    ./launch/roscore.sh \
    /opt/${ORGANIZATION}/
RUN ln -s /opt/${ORGANIZATION}/${PROJECT_NAME} ${HOME}/${PROJECT_NAME}

RUN mkdir -p ${HOME}/.ros && \
    chown -R 1000:1000 ${HOME}/.ros

WORKDIR ${HOME}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
