cmake_minimum_required(VERSION 2.8.3)
project(racecar_agent)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  ackermann_msgs
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy std_msgs sensor_msgs ackermann_msgs
)

catkin_install_python(
  PROGRAMS src/racecar_agent.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  launch/racecar_agent.launch
  models/model.hdf5
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
