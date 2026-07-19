cmake_minimum_required(VERSION 3.0.2)
project(see_point_fly_bridge)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  mavros_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/see_point_fly_bridge.py
  scripts/spf_task_executor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
