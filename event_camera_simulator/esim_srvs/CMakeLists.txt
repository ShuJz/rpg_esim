cmake_minimum_required(VERSION 2.8.3)
project(esim_srvs)

# search for everything we need to build the messages, dont forget the message_generation
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  dvs_msgs
)

# search for all msg files
FILE(GLOB services_to_build RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv"
    "${CMAKE_CURRENT_SOURCE_DIR}/srv/*.srv")

# notify catkin to look at the previously found msg files
add_service_files(
  FILES
  ${services_to_build}
)

# build the header files from the msg files, and notify catkin about the dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  dvs_msgs
)

# export the dependencis of this package for who ever depends on us
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
