cmake_minimum_required(VERSION 2.8.3)

project(group3_lab5)
set(NODE_NAME "scan_matcher" CACHE STRING "")

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs geometry_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs tf2_sensor_msgs
  DEPENDS EIGEN3 Boost
)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(${NODE_NAME}
  src/scan_matcher.cpp
  src/scan_matcher/transform.cpp
  src/scan_matcher/correspond.cpp
  src/scan_matcher/visualization.cpp
)

target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})

install(TARGETS ${NODE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
        launch/scan_matcher.launch
        launch/simulator.launch
        launch/simulator.rviz
        launch/simulator.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_gtest(test_icp_algorithm
        test/test_icp_algorithm.cpp
        src/scan_matcher/transform.cpp
        src/scan_matcher/correspond.cpp
        src/scan_matcher/visualization.cpp
)

target_link_libraries(test_icp_algorithm ${catkin_LIBRARIES})

#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -DGLIBCXX_DEBUG -D_GLIBCXX_DEBUG")
#set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -DGLIBCXX_DEBUG -D_GLIBCXX_DEBUG")

#message("CMAKE_C_FLAGS_DEBUG is ${CMAKE_C_FLAGS_DEBUG}")
#message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")