cmake_minimum_required(VERSION 3.0.2)
project(mcap_dataset_recorder)

find_package(catkin REQUIRED COMPONENTS
  iiwa_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/record_mcap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)