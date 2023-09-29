#!/bin/bash

BUILD_TYPE="Release"
GENERATOR="Unix Makefiles"

SRC_DIR=$(pwd)
EXT_SRC_DIR="${SRC_DIR}/external"
LGMATH_SRC_DIR="${SRC_DIR}/lgmath"
DOPPLER_ODOM_SRC_DIR="${SRC_DIR}/doppler_odom"

BUILD_DIR="${SRC_DIR}/cmake-build-${BUILD_TYPE}"
EXT_BUILD_DIR=$BUILD_DIR/external
LGMATH_BUILD_DIR=$BUILD_DIR/lgmath
DOPPLER_ODOM_BUILD_DIR=$BUILD_DIR/doppler_odom

mkdir -p $BUILD_DIR
mkdir -p $EXT_BUILD_DIR
mkdir -p $LGMATH_BUILD_DIR
mkdir -p $DOPPLER_ODOM_BUILD_DIR

check_status_code() {
	if [ $1 -ne 0 ]; then
		echo "[DOPPLER_ODOM] Failure. Exiting."
		exit 1
	fi
}

echo "[DOPPLER_ODOM] -- [EXTERNAL DEPENDENCIES] -- Generating the cmake project"
cd ${EXT_BUILD_DIR}
cmake -G "$GENERATOR" -S $EXT_SRC_DIR -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
check_status_code $?

echo "[DOPPLER_ODOM] -- [EXTERNAL DEPENDENCIES] -- building CMake Project"
cmake --build . --config $BUILD_TYPE
check_status_code $?

echo "[DOPPLER_ODOM] -- [LGMATH] -- Generating the cmake project"
cd ${LGMATH_BUILD_DIR}
cmake -G "$GENERATOR" -S $LGMATH_SRC_DIR \
	-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
	-DUSE_AMENT=OFF \
	-DEigen3_DIR=${EXT_BUILD_DIR}/install/${BUILD_TYPE}/Eigen3/share/eigen3/cmake \
	-DCMAKE_INSTALL_PREFIX=${LGMATH_BUILD_DIR}/install/${BUILD_TYPE}
check_status_code $?

echo "[DOPPLER_ODOM] -- [LGMATH] -- building CMake Project"
cmake --build . --config $BUILD_TYPE --target install --parallel 6
check_status_code $?

echo "[DOPPLER_ODOM] -- [DOPPLER_ODOM] -- Generating the cmake project"
cd ${DOPPLER_ODOM_BUILD_DIR}
cmake -G "$GENERATOR" -S $DOPPLER_ODOM_SRC_DIR \
	-DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
	-Dlgmath_DIR=${LGMATH_BUILD_DIR}/install/${BUILD_TYPE}/lib/cmake/lgmath \
	-DEigen3_DIR=${EXT_BUILD_DIR}/install/${BUILD_TYPE}/Eigen3/share/eigen3/cmake \
	-Dglog_DIR=${EXT_BUILD_DIR}/install/${BUILD_TYPE}/glog/lib/cmake/glog \
	-Dyaml-cpp_DIR=${EXT_BUILD_DIR}/install/${BUILD_TYPE}/yaml-cpp/share/cmake/yaml-cpp
check_status_code $?

echo "[DOPPLER_ODOM] -- [DOPPLER_ODOM] -- building CMake Project"
cmake --build . --config $BUILD_TYPE --parallel 6
check_status_code $?
