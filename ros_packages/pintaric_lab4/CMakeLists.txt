cmake_minimum_required(VERSION 2.8.3)
project(pintaric_lab4)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  ackermann_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/reactive_driving_.cfg
)

catkin_package(
 CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs visualization_msgs ackermann_msgs dynamic_reconfigure
)

catkin_install_python(
  PROGRAMS src/follow_the_gap.py src/drive_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  launch/follow_the_gap.launch
  launch/simulator.launch
  params/follow_the_gap.perspective
  params/reactive_driving.yaml
  params/simulator.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
