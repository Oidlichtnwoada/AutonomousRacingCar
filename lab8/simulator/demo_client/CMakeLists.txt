cmake_minimum_required(VERSION 3.16)
project(demo_client)

find_package (Python3 COMPONENTS Interpreter)
if(NOT Python3_Interpreter_FOUND)
    set(PYTHON3_INTERPRETER python3)
else()
    set(PYTHON3_INTERPRETER Python3::Interpreter)
endif()

get_filename_component(racecar_simulator_proto "${CMAKE_SOURCE_DIR}/simulator/protos/racecar_simulator.proto" ABSOLUTE)
get_filename_component(racecar_simulator_proto_path "${racecar_simulator_proto}" PATH)

add_custom_target(demo_client_grpc ALL
        COMMAND ${PYTHON3_INTERPRETER} -m grpc_tools.protoc -I ${racecar_simulator_proto_path} --python_out=${demo_client_SOURCE_DIR} --grpc_python_out=${demo_client_SOURCE_DIR} racecar_simulator.proto
        DEPENDS "${racecar_simulator_proto}"
        BYPRODUCTS "${demo_client_SOURCE_DIR}/racecar_simulator_pb2.py" "${demo_client_SOURCE_DIR}/racecar_simulator_pb2_grpc.py"
        )