cmake_minimum_required(VERSION 2.8.3)
project(respeaker_ros)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)

generate_dynamic_reconfigure_options(
  cfg/Respeaker.cfg
)

catkin_package()

install(DIRECTORY scripts launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
