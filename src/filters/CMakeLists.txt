cmake_minimum_required(VERSION 2.8.3)
project(filters)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Volex Grid Filter Node
add_executable (volex_filter src/volex_filter.cpp)
target_link_libraries (volex_filter ${PCL_LIBRARIES})

# PassThrough Filter Node
add_executable (passthrough_filter src/passthrough_filter.cpp)
target_link_libraries (passthrough_filter ${PCL_LIBRARIES})
