#!/bin/bash
# 
# Modal AI Inc. 2021
# author: ali.younis@modalai.com

# Build 
mkdir -p build64
cd build64
# cmake -DCMAKE_BUILD_TYPE=Release  -DBUILD_FOR_X86=ON ../
cmake -DCMAKE_BUILD_TYPE=Release ../
make -j4
cd ../
