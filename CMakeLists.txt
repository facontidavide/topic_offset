cmake_minimum_required(VERSION 2.8.3)
project(topic_distance)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  rosbag_storage
)

catkin_package()

set( CMAKE_BUILD_TYPE Release )

include_directories( ${catkin_INCLUDE_DIRS} )
add_definitions(-std=c++11)

file( GLOB SOURCES src/*.cpp )
add_executable( ${PROJECT_NAME} ${SOURCES} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )
