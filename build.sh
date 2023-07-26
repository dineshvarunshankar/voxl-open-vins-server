#!/bin/bash

## voxl-cross contains the following toolchains
## first two for apq8096, last for qrb5165
TOOLCHAIN_APQ8096_32="/opt/cross_toolchain/arm-gnueabi-4.9.toolchain.cmake"
TOOLCHAIN_APQ8096_64="/opt/cross_toolchain/aarch64-gnu-4.9.toolchain.cmake"
TOOLCHAIN_QRB5165="/opt/cross_toolchain/aarch64-gnu-7.toolchain.cmake"

# placeholder in case more cmake opts need to be added later
EXTRA_OPTS=""

## this list is just for tab-completion
AVAILABLE_PLATFORMS="apq8096 qrb5165 native"

# qrb5165 compiler definition, used for qrb5165 specific usage
BUILD_QRB5165="ON"

print_usage(){
	echo ""
	echo " Build the current project based on platform target."
	echo ""
	echo " Usage:"
	echo ""
	echo "  ./build.sh apq8096"
	echo "        Build 64-bit binaries for apq8096"
	echo ""
	echo "  ./build.sh qrb5165"
	echo "        Build 64-bit binaries for qrb5165"
	echo ""
	echo "  ./build.sh native"
	echo "        Build with the native gcc/g++ compilers."
	echo ""
	echo ""
}

case "$1" in
	apq8096)
		mkdir -p build64
		cd build64
		BUILD_QRB5165="OFF"
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_APQ8096_64} -DBUILD_QRB5165=${BUILD_QRB5165} -DBOOST_ROOT="/usr/include/boost64/" -DCMAKE_BUILD_TYPE=Release ${EXTRA_OPTS} ../
		make -j$(nproc)
		cd ../
		;;
	qrb5165)
		mkdir -p build64
		cd build64
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165} -DBUILD_QRB5165=${BUILD_QRB5165} -DCMAKE_BUILD_TYPE=Release ${EXTRA_OPTS} ../
		make -j$(nproc) 
		cd ../
		;;

	native)
		mkdir -p build
		cd build
		cmake ${EXTRA_OPTS}  -DCMAKE_BUILD_TYPE=Release ../
		make -j$(nproc)
		cd ../
		;;

	*)
		print_usage
		exit 1
		;;
esac

