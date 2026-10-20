cmake_minimum_required(VERSION 4.1.1)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_COLOR_DIAGNOSTICS ON)
project(high_level_controller)

file(
        DOWNLOAD
        https://github.com/cpm-cmake/CPM.cmake/releases/download/v0.40.8/CPM.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake
        EXPECTED_HASH SHA256=78ba32abdf798bc616bab7c73aac32a17bbd7b06ad9e26a6add69de8f3ae4791
)
include(${CMAKE_CURRENT_BINARY_DIR}/cmake/CPM.cmake)

# Build against the pipenv's interpreter so the module's ABI matches the env that
# setup.py installs into. Turn off to pick an interpreter with -DPython_EXECUTABLE.
option(USE_PIPENV_PYTHON "Build against the Python of the enclosing pipenv" ON)
if (USE_PIPENV_PYTHON)
    find_program(PIPENV_EXECUTABLE pipenv)
    if (PIPENV_EXECUTABLE)
        # Inside `pipenv shell`/`pipenv run` this returns the active env, otherwise
        # the env of the nearest Pipfile above this directory.
        execute_process(
                COMMAND ${PIPENV_EXECUTABLE} --py
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                OUTPUT_VARIABLE PIPENV_PYTHON
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET
                RESULT_VARIABLE PIPENV_RESULT
        )
    endif ()
    if (PIPENV_EXECUTABLE AND PIPENV_RESULT EQUAL 0 AND EXISTS "${PIPENV_PYTHON}")
        execute_process(
                COMMAND ${PIPENV_PYTHON} --version
                OUTPUT_VARIABLE PIPENV_PYTHON_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        message(STATUS "Using pipenv Python: ${PIPENV_PYTHON} (${PIPENV_PYTHON_VERSION})")
        # A pipenv can be recreated in place with a different Python, which neither
        # FindPython nor pybind11 notice, so drop their cached results on any change.
        set(PIPENV_PYTHON_ID "${PIPENV_PYTHON} ${PIPENV_PYTHON_VERSION}")
        if (NOT PIPENV_PYTHON_ID STREQUAL PIPENV_PYTHON_ID_LAST)
            get_cmake_property(cache_vars CACHE_VARIABLES)
            list(FILTER cache_vars INCLUDE REGEX "^_?Python_|^PYTHON_(IS_DEBUG|MODULE_)")
            foreach (var IN LISTS cache_vars)
                unset(${var} CACHE)
            endforeach ()
            set(PIPENV_PYTHON_ID_LAST "${PIPENV_PYTHON_ID}" CACHE INTERNAL "")
        endif ()
        # FORCE so a previously cached interpreter doesn't win over the pipenv one.
        set(Python_EXECUTABLE "${PIPENV_PYTHON}" CACHE FILEPATH "Python interpreter" FORCE)
    else ()
        message(WARNING "Could not locate a pipenv Python, falling back to FindPython's default")
    endif ()
endif ()

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
add_library(CDT_interface INTERFACE)
target_include_directories(CDT_interface INTERFACE ${CDT_SOURCE_DIR}/CDT/include)

CPMAddPackage(
        NAME EIGEN
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen
        GIT_TAG 3147391d946bb4b6c68edd901f2add6ac1f31f8c
        GIT_SHALLOW TRUE
        DOWNLOAD_ONLY TRUE
)
set(EIGEN_INCLUDE_DIRS ${EIGEN_SOURCE_DIR})
# Keep Eigen external to ProSuite's install export set.
if (NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
    set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIRS}"
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${EIGEN_INCLUDE_DIRS}"
            INTERFACE_COMPILE_DEFINITIONS
            "EIGEN_STACK_ALLOCATION_LIMIT=0;EIGEN_MAX_STATIC_ALIGN_BYTES=0"
    )
endif ()

CPMAddPackage(
        NAME AUTO_DIFF
        GITHUB_REPOSITORY autodiff/autodiff
        GIT_TAG 2e2f3a2b16afcd9c04e76c8a689e9fd23ff78679
        GIT_SHALLOW TRUE
        DOWNLOAD_ONLY TRUE
)
set(AUTO_DIFF_INCLUDE_DIRS
        # Required by autodiff internal includes like <autodiff/common/...>
        "${AUTO_DIFF_SOURCE_DIR}"
        # so we can directly include <dual.hpp>
        "${AUTO_DIFF_SOURCE_DIR}/autodiff/forward"
        # so we can directly include <gradient.hpp>
        "${AUTO_DIFF_SOURCE_DIR}/autodiff/forward/utils"
)

add_library(autodiff_interface INTERFACE)
target_link_libraries(autodiff_interface INTERFACE Eigen3::Eigen)
target_include_directories(autodiff_interface SYSTEM INTERFACE ${AUTO_DIFF_INCLUDE_DIRS})

CPMAddPackage(
        NAME alglib
        URL https://www.alglib.net/translator/re/alglib-4.07.0.cpp.gpl.zip
        DOWNLOAD_ONLY TRUE
)
add_library(alglib_interface INTERFACE)
target_include_directories(alglib_interface INTERFACE ${alglib_SOURCE_DIR}/src)

# Use the source archive (not the amalgamated one) so headers live under simde/
CPMAddPackage(
        NAME simde
        URL https://github.com/simd-everywhere/simde/archive/refs/tags/v0.8.2.tar.gz
        DOWNLOAD_ONLY TRUE
)
if (NOT TARGET simde)
    add_library(simde INTERFACE IMPORTED GLOBAL)
    set_target_properties(simde PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${simde_SOURCE_DIR}"
            INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${simde_SOURCE_DIR}"
    )
endif ()

# Tell proxsuite's jrl-cmakemodules that Eigen3 and Simde targets are provided
# here, so it skips find_package() for them.
set(PROJECT_PACKAGES_IN_WORKSPACE Eigen3 Simde)
CPMAddPackage(
        NAME proxsuite
        URL https://github.com/Simple-Robotics/proxsuite/releases/download/v0.7.3/proxsuite-0.7.3.tar.gz
        OPTIONS "BUILD_TESTING OFF"
)

file(GLOB ALGLIB_SOURCES ${alglib_SOURCE_DIR}/src/*.cpp)
file(GLOB SOURCES *.cpp)
pybind11_add_module(controller ${SOURCES} ${ALGLIB_SOURCES})
target_link_libraries(controller PRIVATE CDT_interface Eigen3::Eigen alglib_interface autodiff_interface proxsuite)


add_executable(test
        controller.cpp
        perception.cpp
        path.cpp
        sim.cpp
        test.cpp
        tire_model.cpp
        ${ALGLIB_SOURCES}
)
target_link_libraries(test PRIVATE CDT_interface Eigen3::Eigen alglib_interface autodiff_interface proxsuite)
