# -----------------------------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@gmail.com)
# SPDX-License-Identifier: GPL-3.0-or-later
# -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(f1tenth_simulator)

# Required due to multiple 'uninstall' targets in external libraries:
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS TRUE)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(FETCHCONTENT_BASE_DIR ${CMAKE_BINARY_DIR}/_dependencies)

FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp
        GIT_TAG        yaml-cpp-0.6.3
)
option(YAML_CPP_BUILD_TESTS OFF)
option(YAML_CPP_BUILD_TOOLS OFF)
option(YAML_CPP_BUILD_CONTRIB ON)
option(YAML_CPP_INSTALL OFF)

FetchContent_Declare(
        octomap
        GIT_REPOSITORY git@github.com:pintaric/octomap.git
        GIT_TAG        0132ff27f18ea218315395f7d76cb8a6b8eb6be7
        SOURCE_SUBDIR  octomap
)
option(BUILD_OCTOVIS_SUBPROJECT OFF)
option(BUILD_DYNAMICETD3D_SUBPROJECT OFF)
option(OCTOVIS_QT5 OFF)
add_compile_definitions(OCTOMAP_NODEBUGOUT)
add_compile_definitions(OCTOMAP_NOWARNINGOUT)

FetchContent_Declare(
        tclap
        GIT_REPOSITORY https://github.com/mirror/tclap
        GIT_TAG        v1.2.2
)

FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest
        GIT_TAG        release-1.10.0
)

FetchContent_Declare(
        gRPC
        GIT_REPOSITORY https://github.com/grpc/grpc
        GIT_TAG        v1.29.1
)

FetchContent_Declare(
        json
        GIT_REPOSITORY https://github.com/nlohmann/json
        GIT_TAG        v3.7.3
)

FetchContent_MakeAvailable(octomap)
FetchContent_MakeAvailable(json)
FetchContent_MakeAvailable(yaml-cpp)
FetchContent_MakeAvailable(tclap)
FetchContent_MakeAvailable(googletest)
FetchContent_MakeAvailable(gRPC)

add_subdirectory(map_converter)
add_subdirectory(simulator)
add_subdirectory(demo_client)