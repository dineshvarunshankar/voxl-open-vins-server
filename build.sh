#!/bin/bash
# 
# Modal AI Inc. 2021
# author: ali.younis@modalai.com


# Set the toolchain
TOOLCHAIN64="/opt/cross_toolchain/aarch64-gnu-4.9.toolchain.cmake"

# Build 
mkdir -p build64
cd build64
# cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN64} -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/lib64,/usr/lib ../
# cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN64} -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR="lib64"  -DCMAKE_INSTALL_PREFIX=/usr ../
# cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN64} -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_LIBDIR="lib64"  ../
# cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN64} -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_LIBDIR="lib64"  ../
# cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN64} -DCMAKE_BUILD_TYPE=Release  ../
cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN64} -DCMAKE_BUILD_TYPE=Release  ../
make -j4
cd ../
