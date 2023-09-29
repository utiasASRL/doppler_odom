cmake_minimum_required(VERSION 3.14)
project(DOPPLER_ODOM-EXTERNAL)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)
if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

set(EXT_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/install/${CMAKE_BUILD_TYPE})
set(GLOBAL_RUNTIME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

# INSTALL GLOG
ExternalProject_Add(
	glog
	PREFIX glog

	GIT_REPOSITORY https://github.com/google/glog
	GIT_TAG v0.5.0

	BUILD_ALWAYS OFF
	INSTALL_DIR ${EXT_INSTALL_DIR}/glog
	CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX=${EXT_INSTALL_DIR}/glog
		-DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}

	STEP_TARGETS 	  install
	BUILD_COMMAND     ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)

# INSTALL EIGEN
ExternalProject_Add(
	Eigen3_ext
	PREFIX Eigen3_ext

	GIT_REPOSITORY  https://gitlab.com/libeigen/eigen
	GIT_TAG 3.3.7

	BUILD_ALWAYS OFF
	INSTALL_DIR ${EXT_INSTALL_DIR}/Eigen3
	CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX=${EXT_INSTALL_DIR}/Eigen3
		-DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}

	STEP_TARGETS 	install
	BUILD_COMMAND     ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --parallel 6)

# Install yaml-cpp
ExternalProject_Add(
	yaml-cpp
	PREFIX yaml-cpp

	GIT_REPOSITORY	https://github.com/jbeder/yaml-cpp
	GIT_TAG yaml-cpp-0.7.0

	BUILD_ALWAYS OFF
	INSTALL_DIR ${EXT_INSTALL_DIR}/yaml-cpp
	CMAKE_ARGS
		-DCMAKE_INSTALL_PREFIX=${EXT_INSTALL_DIR}/yaml-cpp
		-DCMAKE_CXX_STANDARD=${CMAKE_CXX_STANDARD}

	DEPENDS
	BUILD_COMMAND     ${CMAKE_COMMAND} --build <BINARY_DIR> --config ${CMAKE_BUILD_TYPE} --target install --parallel 6)
