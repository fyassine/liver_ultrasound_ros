cmake_minimum_required(VERSION 3.0.2)
project(sensor_dataset_recorder)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  iiwa_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/record_mcap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
