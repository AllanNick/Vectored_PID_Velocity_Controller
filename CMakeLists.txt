cmake_minimum_required(VERSION 3.10)
project(VPVController_Mark_I)

set(CMAKE_CXX_STANDARD 11)
include_directories(${CMAKE_SOURCE_DIR}/Inc)

add_executable(a ${CMAKE_SOURCE_DIR}/Src/main.c)
