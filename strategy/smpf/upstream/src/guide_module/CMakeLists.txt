cmake_minimum_required(VERSION 3.0.2)
project(guide_module)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  ros_interface/ROSInterFace.py
  GuideNode.py
  guide_panel/PanelWeb.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
