cmake_minimum_required(VERSION 3.6)
project(cis_cal)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp)
add_executable(cis_cal ${SOURCE_FILES})