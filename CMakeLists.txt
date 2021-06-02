cmake_minimum_required(VERSION 3.5)
project(GIOW)
set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE "Release")

include_directories(${PROJECT_SOURCE_DIR}/lib/eigen3)
IF (WIN32)
    link_directories(${PROJECT_SOURCE_DIR}/lib/win)
ELSE ()
    link_directories(${PROJECT_SOURCE_DIR}/lib/linux)
ENDIF ()

add_executable(GIOW src/main.cpp src/vel_meas.cpp)
TARGET_LINK_LIBRARIES(GIOW liblc.a)
