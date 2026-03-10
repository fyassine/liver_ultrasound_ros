cmake_minimum_required(VERSION 3.0.2)
project(rear_mic_recorder)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rear_mic_recorder_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
