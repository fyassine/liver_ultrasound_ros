cmake_minimum_required(VERSION 3.0.2)
project(capture_clarius_ultrasound)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/clarius_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/libcast.so
  scripts/pyclariuscast.so
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)