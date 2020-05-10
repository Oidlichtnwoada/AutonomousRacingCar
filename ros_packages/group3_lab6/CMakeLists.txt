cmake_minimum_required(VERSION 2.8.3)

project(group3_lab6)

set(NODE_NAME "dummy" CACHE STRING "")

set(CMAKE_CXX_STANDARD 17)

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
  src/${NODE_NAME}.cpp
)

target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})

install(TARGETS ${NODE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
        launch/simulator.launch
        launch/simulator.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

