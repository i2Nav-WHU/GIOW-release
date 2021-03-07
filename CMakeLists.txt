cmake_minimum_required(VERSION 3.5)
project(GIOW)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(GIOW src/main.cpp src/vel_meas.cpp)
TARGET_LINK_LIBRARIES(GIOW liblc.a)
