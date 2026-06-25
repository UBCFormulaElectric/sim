cmake_minimum_required(VERSION 4.1.1)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
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

CPMAddPackage(
        NAME CDT
        GIT_REPOSITORY https://github.com/artem-ogre/CDT.git
        GIT_TAG 1.4.4
        DOWNLOAD_ONLY TRUE
)
add_library(cdt_interface INTERFACE)
target_include_directories(cdt_interface SYSTEM INTERFACE ${CDT_SOURCE_DIR}/CDT/include)

CPMAddPackage(
        NAME EIGEN
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen
        GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c
        GIT_SHALLOW TRUE
)
set(EIGEN_INCLUDE_DIRS ${EIGEN_SOURCE_DIR})
add_library(eigen_interface INTERFACE)
target_include_directories(eigen_interface SYSTEM INTERFACE ${EIGEN_INCLUDE_DIRS})
target_compile_definitions(eigen_interface INTERFACE
        EIGEN_STACK_ALLOCATION_LIMIT=0
        EIGEN_MAX_STATIC_ALIGN_BYTES=0
)

CPMAddPackage(
        NAME alglib
        URL https://www.alglib.net/translator/re/alglib-4.07.0.cpp.gpl.zip
        DOWNLOAD_ONLY TRUE
)
add_library(alglib_interface INTERFACE)
target_include_directories(alglib_interface INTERFACE ${alglib_SOURCE_DIR}/src)

CPMAddPackage(
        NAME simde
        URL https://github.com/simd-everywhere/simde/releases/download/v0.8.2/simde-amalgamated-0.8.2.tar.xz
)

CPMAddPackage(
        NAME proxsuite
        URL https://github.com/Simple-Robotics/proxsuite/releases/download/v0.7.3/proxsuite-0.7.3.tar.gz
)

file(GLOB SOURCES *.cpp ${alglib_SOURCE_DIR}/src/*.cpp)
pybind11_add_module(controller ${SOURCES})
target_link_libraries(controller PRIVATE eigen_interface alglib_interface cdt_interface proxsuite)