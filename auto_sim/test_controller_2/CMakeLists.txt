cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 23)
project(high_level_controller)

file(
  DOWNLOAD
  https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.8/CPM.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
  EXPECTED_HASH SHA256=78ba32abdf798bc616bab7c73aac32a17bbd7b06ad9e26a6add69de8f3ae4791
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

CPMAddPackage(
  NAME pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  VERSION 3.0.2
  DOWNLOAD_ONLY TRUE
)
add_subdirectory(${pybind11_SOURCE_DIR})

file(GLOB SOURCES *.cpp)
pybind11_add_module(controller ${SOURCES})