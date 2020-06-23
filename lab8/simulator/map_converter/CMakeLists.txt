# -----------------------------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@gmail.com)
# SPDX-License-Identifier: GPL-3.0-or-later
# -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(map_converter)

set(CMAKE_CXX_STANDARD 20)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED
        HINTS ${CMAKE_BINARY_DIR}/_dependencies/octomap-src/octomap/lib/cmake/octomap
        ${CMAKE_BINARY_DIR}/_dependencies/octomap-src/octomap/lib/cmake/octomap
        )
MESSAGE(STATUS "Found octomap version: " ${octomap_VERSION})
MESSAGE(STATUS "octomap libraries: ${OCTOMAP_LIBRARIES}")
INCLUDE_DIRECTORIES(BEFORE SYSTEM ${OCTOMAP_INCLUDE_DIRS})

include_directories(
        "${PROJECT_SOURCE_DIR}/include"
)

file(GLOB SOURCE_FILES
        "${PROJECT_SOURCE_DIR}/include/*.h"
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
)

add_executable(map_converter
        ${SOURCE_FILES}
)

target_include_directories(map_converter PUBLIC
        include
        ${tclap_SOURCE_DIR}/include
        ${YAML_CPP_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
)

add_dependencies(map_converter octomap yaml-cpp)
target_link_libraries(map_converter octomap-static octomath-static yaml-cpp ${OpenCV_LIBS})

if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
    target_compile_options(map_converter PRIVATE -lstdc++fs)
    target_link_libraries(map_converter stdc++fs)
endif()

install(TARGETS map_converter 
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        CONFIGURATIONS Release)
