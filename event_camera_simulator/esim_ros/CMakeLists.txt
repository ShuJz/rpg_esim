cmake_minimum_required(VERSION 2.8.3)
project(esim_ros)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(catkin REQUIRED COMPONENTS
# roscpp
# rospy
# std_msgs
# message_generation)
catkin_simple()

set(HEADERS
    include/params.hpp
)

set(SOURCES
    # src/esim_node.cpp
    src/esim_server.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# make the executable
# cs_add_executable(esim_node  src/esim_node.cpp)

# # link the executable to the necessary libs
# target_link_libraries(esim_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

cs_add_executable(esim_server  src/esim_server.cpp)

# link the executable to the necessary libs
target_link_libraries(esim_server ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_install_python(PROGRAMS scripts/esim_server_client_tast.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cs_install()
