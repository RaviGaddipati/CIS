cmake_minimum_required(VERSION 3.4)
project(cis_icp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -pedantic -Wall -Wextra -Wno-unused-function -Wno-sign-compare")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(eigen/Eigen include doctest/doctest )

set(SOURCE_FILES
        src/main.cpp
        include/files.h
        src/files.cpp
        include/pointcloud.h
        src/utils.cpp
        include/utils.h
        include/icp.h
        src/icp.cpp
        include/registration.h
        src/registration.cpp
        src/horn.cpp
        include/horn.h
        src/surface.cpp
        include/surface.h)

add_executable(cisicp ${SOURCE_FILES})