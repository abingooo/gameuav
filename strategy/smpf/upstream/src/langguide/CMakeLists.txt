cmake_minimum_required(VERSION 3.0.2)
project(langguide)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/langguide_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
