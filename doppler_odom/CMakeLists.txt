cmake_minimum_required(VERSION 3.16)
project(doppler_odom)

# Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Turn on as many warnings as possible by default.
# add_compile_options(-march=native -O3 -pthread -Wall -Wextra)
add_compile_options(-march=native -O3 -Wall -Wextra)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
# find_package(OpenMP REQUIRED)
find_package(lgmath REQUIRED)
find_package(yaml-cpp REQUIRED)

# Force eigen to not parallelize
add_compile_definitions(EIGEN_DONT_PARALLELIZE)

include_directories(
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

link_libraries(
  glog::glog Eigen3::Eigen
  lgmath yaml-cpp
)

file(GLOB_RECURSE DOPPLER_ODOM_SRCS src/*.cpp)
add_executable(${PROJECT_NAME} ${DOPPLER_ODOM_SRCS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)
