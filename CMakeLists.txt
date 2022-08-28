# Project

cmake_minimum_required(VERSION 3.19)
project(voxelizer)
set(CMAKE_CXX_STANDARD 17)

# Integration tests

set(TEST_TARGET ${CMAKE_PROJECT_NAME}-tests)

file(GLOB TEST_SOURCES test/*.cpp)
add_executable(${TEST_TARGET} ${TEST_SOURCES})

# C99 compliance

set (C99_TARGET ${CMAKE_PROJECT_NAME}-c99)

add_executable(${C99_TARGET} example/voxelizer.c)
set_property(TARGET ${C99_TARGET} PROPERTY C_STANDARD 99)
