# ------------------------------------------------------------------------------------------------------------
# Author: Wenhao Zhao (wenhao.zhao@tum.de)
# Version: Feb 2026
# Description: Dockerfile for franka_haptic_teleoperation package, based on ROS Noetic and libfranka 0.18.0
# ------------------------------------------------------------------------------------------------------------

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
SHELL ["/bin/bash", "-c"]

# ---------------------------
# Base deps
# ---------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg2 lsb-release \
    build-essential cmake git pkg-config \
    python3 python3-pip \
    libpoco-dev libeigen3-dev libssl-dev libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------
# robotpkg repo (Pinocchio on Ubuntu 20.04)
# ---------------------------
RUN echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub focal robotpkg" \
    > /etc/apt/sources.list.d/robotpkg.list && \
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add -

RUN apt-get update && apt-get install -y --no-install-recommends \
    robotpkg-pinocchio \
    && rm -rf /var/lib/apt/lists/*

ENV CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
ENV PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
ENV LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH

# ---------------------------
# ROS Noetic
# ---------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
    > /etc/apt/sources.list.d/ros1.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep python3-catkin-tools python3-vcstool \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-combined-robot-hw \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# ---------------------------
# libfranka (force C++17 at build time)
# ---------------------------
ARG LIBFRANKA_TAG=0.18.0
WORKDIR /opt
RUN git clone --recurse-submodules https://github.com/frankarobotics/libfranka.git && \
    cd libfranka && \
    git checkout ${LIBFRANKA_TAG} && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DBUILD_TESTS=OFF \
          -DCMAKE_CXX_STANDARD=17 \
          -DCMAKE_CXX_STANDARD_REQUIRED=ON \
          -DCMAKE_PREFIX_PATH="/opt/openrobots" .. && \
    cmake --build . -j"$(nproc)" && \
    cmake --install .

# ---------------------------
# boost_sml (header-only + minimal CMake config)
# ---------------------------
ARG BOOST_SML_TAG=v1.1.11
WORKDIR /opt
RUN git clone https://github.com/boost-ext/sml.git && \
    cd sml && \
    git checkout ${BOOST_SML_TAG} && \
    mkdir -p /usr/local/include/boost && \
    cp -r include/boost/sml /usr/local/include/boost/ && \
    mkdir -p /usr/local/lib/cmake/boost_sml && \
    printf '%s\n' \
      'add_library(boost_sml::boost_sml INTERFACE IMPORTED)' \
      'set_target_properties(boost_sml::boost_sml PROPERTIES' \
      '  INTERFACE_INCLUDE_DIRECTORIES "/usr/local/include"' \
      ')' \
      > /usr/local/lib/cmake/boost_sml/boost_smlConfig.cmake

# ---------------------------
# franka_ros workspace
# ---------------------------
ARG FRANKA_ROS_BRANCH=noetic-devel
WORKDIR /ws/src
RUN git clone -b ${FRANKA_ROS_BRANCH} https://github.com/frankarobotics/franka_ros.git

# Patch franka_ros to force C++17
RUN set -e; \
    cd /ws/src/franka_ros; \
    grep -RIl "CMAKE_CXX_STANDARD" . | while read -r f; do \
      sed -i 's/CMAKE_CXX_STANDARD[[:space:]]*11/CMAKE_CXX_STANDARD 17/g' "$f"; \
      sed -i 's/CMAKE_CXX_STANDARD[[:space:]]*14/CMAKE_CXX_STANDARD 17/g' "$f"; \
    done
