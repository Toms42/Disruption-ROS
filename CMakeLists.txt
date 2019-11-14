cmake_minimum_required(VERSION 2.8.3)
project(rcp_spotlight)

add_compile_options(-Wall -Werror -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  sensor_msgs
  geometry_msgs
  message_filters
  tf
  vrpn_client_ros
  message_generation
  )

catkin_python_setup()


add_service_files(
  FILES
  SetInt32.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  rospy
  sensor_msgs
  geometry_msgs
  tf
  message_filters
  vrpn_client_ros
  )

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*~" EXCLUDE)


# define nodes here
# eg: add_subdirectory(src/imu_calibrator)


