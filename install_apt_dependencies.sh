#!/usr/bin/bash

APT_DEPENDENCIES="cmake \
                  g++ \
                  libavcodec-dev \
                  libavdevice-dev \
                  libavfilter-dev \
                  libavformat-dev \
                  libavutil-dev \
                  libblas-dev \
                  libboost-serialization-dev \
                  libc++-dev \
                  libegl1-mesa-dev \
                  libepoxy-dev \
                  libfmt-dev \
                  libgl1-mesa-dev \
                  libgles2-mesa-dev \
                  libglew-dev \
                  liblapack-dev \
                  libopencv-dev \
                  libspdlog-dev \
                  libswresample-dev \
                  libswscale-dev \
                  libwayland-dev \
                  libx11-dev \
                  libxkbcommon-dev \
                  nasm \
                  ninja-build \
                  wayland-protocols"


myarg=$1
if [[ "$myarg" = "--deps" ]]; then
    echo $APT_DEPENDENCIES
    exit 0
fi


sudo apt-get update && \
sudo apt-get install --no-install-recommends -y $APT_DEPENDENCIES
