# -----------------------------------------------------------------------------
# Author: Thomas Pintaric (thomas.pintaric@gmail.com)
# SPDX-License-Identifier: GPL-3.0-or-later
# -----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)
project(racecar_simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(OpenMP REQUIRED)
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
        "${PROJECT_SOURCE_DIR}/src/racecar_simulator/*.cpp"
)

get_filename_component(racecar_simulator_proto "${CMAKE_SOURCE_DIR}/simulator/protos/racecar_simulator.proto" ABSOLUTE)
get_filename_component(racecar_simulator_proto_path "${racecar_simulator_proto}" PATH)

set(racecar_simulator_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/racecar_simulator.pb.cc")
set(racecar_simulator_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/racecar_simulator.pb.h")
set(racecar_simulator_grpc_srcs  "${CMAKE_CURRENT_BINARY_DIR}/racecar_simulator.grpc.pb.cc")
set(racecar_simulator_grpc_hdrs  "${CMAKE_CURRENT_BINARY_DIR}/racecar_simulator.grpc.pb.h")

set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:grpc_cpp_plugin>)

add_custom_command(
        OUTPUT "${racecar_simulator_proto_srcs}" "${racecar_simulator_proto_hdrs}" "${racecar_simulator_grpc_srcs}" "${racecar_simulator_grpc_hdrs}"
        COMMAND ${_PROTOBUF_PROTOC}
        ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}" --cpp_out "${CMAKE_CURRENT_BINARY_DIR}" -I "${racecar_simulator_proto_path}" --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}" "${racecar_simulator_proto}"
        DEPENDS "${racecar_simulator_proto}" grpc++
)

include_directories("${CMAKE_CURRENT_BINARY_DIR}")

add_executable(racecar_simulator
        ${SOURCE_FILES}
        ${racecar_simulator_proto_srcs}
        ${racecar_simulator_grpc_srcs}
)

target_include_directories(racecar_simulator PUBLIC
        include
        ${NLOHMANN_JSON_INCLUDE_BUILD_DIR}
        ${tclap_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_dependencies(racecar_simulator nlohmann_json octomap grpc++)
target_link_libraries(racecar_simulator nlohmann_json octomap-static octomath-static grpc++ OpenMP::OpenMP_CXX ${OpenCV_LIBS} Boost::iostreams)
target_compile_options(racecar_simulator PRIVATE "${OpenMP_CXX_FLAGS}")

if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
    target_compile_options(racecar_simulator PRIVATE -lstdc++fs)
    target_link_libraries(racecar_simulator stdc++fs)
endif()