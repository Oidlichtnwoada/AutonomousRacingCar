cmake_minimum_required(VERSION 2.8.3)
project(group3_lab6)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  sensor_msgs
  geometry_msgs
  visualization_msgs
  ackermann_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/drive_controller_params.cfg
  cfg/pure_pursuit_params.cfg
)

catkin_package(
 CATKIN_DEPENDS rospy std_msgs nav_msgs tf2 tf2_ros sensor_msgs geometry_msgs visualization_msgs ackermann_msgs dynamic_reconfigure
)

catkin_install_python(
  PROGRAMS src/pure_pursuit.py src/drive_controller.py src/waypoint_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  launch/simple_simulator.launch
  launch/simple_simulator.yaml
  launch/simple_simulator.rviz
  launch/pure_pursuit.launch
  launch/drive_controller.yaml
  launch/plotjuggler_layout.xml
  waypoint_logs/waypoint_log.csv
  waypoint_logs/waypoints_resampled.csv
  waypoint_logs/waypoints_lowpass_filtered.csv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
