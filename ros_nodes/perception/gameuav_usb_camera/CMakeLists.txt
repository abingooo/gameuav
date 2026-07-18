cmake_minimum_required(VERSION 3.0.2)
project(gameuav_usb_camera)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs)
find_package(realsense2 QUIET)

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/realsense_nodelet_prelaunch.py
    scripts/usb_camera_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(realsense2_FOUND)
  add_executable(realsense_hw_reset src/realsense_hw_reset.cpp)
  target_link_libraries(realsense_hw_reset ${realsense2_LIBRARY})
  target_include_directories(realsense_hw_reset PRIVATE ${realsense2_INCLUDE_DIR})
  install(TARGETS realsense_hw_reset
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
else()
  message(WARNING "librealsense2 was not found; realsense_hw_reset will not be built")
endif()
