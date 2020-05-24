cmake_minimum_required(VERSION 2.8.3)

project(group3_lab7)
set(NODE_NAME "motion_planner" CACHE STRING "")

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  ackermann_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/follow_me.cfg
)

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs std_srvs sensor_msgs geometry_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs tf2_sensor_msgs ackermann_msgs dynamic_reconfigure
  DEPENDS EIGEN3 Boost
)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

add_executable(${NODE_NAME}
  src/motion_planner.cpp
  src/motion_planner/path_planner.cpp
  src/motion_planner/occupancy_grid.cpp)

target_link_libraries(${NODE_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${NODE_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS src/follow_me.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES 
        launch/motion_planner.launch
        launch/headless_simulator.launch
        launch/rviz.launch
        launch/occupancy_grids.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
