#!/bin/bash

# 32 and 64-bit toolchains available in voxl-cross 4
TOOLCHAIN_QRB5165_1_32="/opt/cross_toolchain/qrb5165_ubun1_18.04_arm32.toolchain.cmake"
TOOLCHAIN_QRB5165_1_64="/opt/cross_toolchain/qrb5165_ubun1_18.04_aarch64.toolchain.cmake"

# placeholder in case more cmake opts need to be added later
EXTRA_OPTS=""

## this list is just for tab-completion
AVAILABLE_PLATFORMS="qrb5165 native"

# qrb5165 compiler definition, used for qrb5165 specific usage
BUILD_QRB5165="ON"

print_usage(){
	echo ""
	echo " Build the current project based on platform target."
	echo ""
	echo " Usage:"
	echo ""
	echo "  ./build.sh qrb5165"
	echo "        Build 64-bit binaries for qrb5165"
	echo ""
	echo "  ./build.sh native"
	echo "        Build with the native gcc/g++ compilers."
	echo ""
	echo ""
}


check_docker() {
	local MIN_VERSION="$1"
	local FILE="/etc/modalai/image.name"

	if [[ ! -f "$FILE" ]]; then
		echo "$FILE does not exist, are you running in the voxl-cross docker?"
		exit 1
	fi

	local IMAGE_STRING
	IMAGE_STRING=$(<"$FILE")

	if [[ "$IMAGE_STRING" =~ ^voxl-cross\(([0-9]+\.[0-9]+)\)$ ]]; then
		local VERSION="${BASH_REMATCH[1]}"
		echo "Found voxl-cross version: $VERSION"

		if [[ "$(printf '%s\n' "$VERSION" "$MIN_VERSION" | sort -V | head -n1)" == "$MIN_VERSION" ]]; then
			return 0
		else
			echo "voxl-cross $VERSION does not meet minimum required $MIN_VERSION."
			exit 1
		fi
	else
		echo "voxl-cross not found in $FILE"
		echo "are you running in the voxl-cross docker?"
		exit 1
	fi
}


case "$1" in
	qrb5165)
		check_docker "4.0"
		mkdir -p build64
		cd build64
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165_1_64} \
				-DPLATFORM=QRB5165 \
				-DEN_ION_BUF=ON \
				-DCMAKE_BUILD_TYPE=Release \
				${EXTRA_OPTS} \
				../
		make -j$(nproc) 
		cd ../
		;;

	native)
		check_docker "4.0"
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

