cmake_minimum_required(VERSION 3.4)
project(cis_cal)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -pedantic -Wall -Wextra -Wno-unused-function -Wno-sign-compare")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(eigen/Eigen include doctest/doctest)

set(SOURCE_FILES
        src/main.cpp
        src/utils.cpp
        src/bernstein.cpp
        src/distortion_calibration.cpp
        src/files.cpp
        src/horn.cpp
        src/pivot_calibration.cpp)

add_executable(ciscal ${SOURCE_FILES})