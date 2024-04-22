FROM ubuntu:20.04

ARG PROJECT_NAME
ARG ORGANIZATION

ENV ROS_DISTRO=noetic
ENV PROJECT_NAME=${PROJECT_NAME}
ENV ORGANIZATION=${ORGANIZATION}
ENV PYTHON_INSTALL_VERSION=3.11
ENV UBUNTU_VERSION_MAJOR=20
ENV UBUNTU_VERSION_MINOR=04

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

ENV TZ=America/New_York

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
    ./install/install_platformio.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_platformio.sh

COPY --chown=1000:1000 \
    ./install/install_apriltag.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_apriltag.sh

RUN sudo mkdir -p ${HOME}/.local && sudo chown -R 1000:1000 ${HOME}/.local

# ---
# ROS base workspace
# ---

ENV PATH=${HOME}/.local/bin${PATH:+:${PATH}}

# ---
# ROS dependency workspace
# ---

COPY --chown=1000:1000 \
    ./install/${ROS_DISTRO}.rosinstall \
    ./install/install_ros.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_ros.sh

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws

COPY --chown=1000:1000 \
    ./install/install_ros_deps.sh \
    ./install/patches/geometry2.patch \
    ./install/patches/image-pipeline.patch \
    ./install/${PROJECT_NAME}.rosinstall \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_ros_deps.sh /opt/${ORGANIZATION}/install/${PROJECT_NAME}.rosinstall

# ---
# Python extra packages
# ---

COPY --chown=1000:1000 \
    ./install/ros-extra-requirements.txt \
    ./install/install_ros_python_extras.sh \
    /opt/${ORGANIZATION}/install/
RUN bash /opt/${ORGANIZATION}/install/install_ros_python_extras.sh

# ---
# Environment variables
# ---

ENV DEP_ROS_WS_ROOT=${HOME}/dep_ws
ENV DEP_ROS_WS_SRC=${HOME}/dep_ws/src

ENV ROS_WS_ROOT=${HOME}/ros_ws
ENV ROS_WS_SRC=${ROS_WS_ROOT}/src

ENV PATH=/opt/${ORGANIZATION}/scripts${PATH:+:${PATH}} \
    PYTHONPATH=/opt/${ORGANIZATION}/${PROJECT_NAME}/ros_ws/bw_tools:/opt/${ORGANIZATION}/${PROJECT_NAME}/shared${PYTHONPATH:+:${PYTHONPATH}} \
    PYTHONIOENCODING=utf-8 \
    PLATFORMIO_CORE_DIR=${ROS_WS_ROOT}/.platformio \
    HISTCONTROL=ignoreboth:erasedups


# ---
# launch environment
# ---

COPY --chown=1000:1000 ./install/bashrc/base_bashrc ${HOME}/.base_bashrc
COPY --chown=1000:1000 ./install/bashrc/ros_bashrc ${HOME}/.bashrc

COPY --chown=1000:1000 \
    ./launch/set_log_format.sh \
    ./launch/launch.sh \
    ./launch/roscore.sh \
    ./launch/rosconsole.config \
    /opt/${ORGANIZATION}/

RUN mkdir -p ${HOME}/.ros && \
    chown -R 1000:1000 ${HOME}/.ros

WORKDIR /opt/${ORGANIZATION}/${PROJECT_NAME}

ENTRYPOINT ["/usr/bin/dumb-init", "--"]
