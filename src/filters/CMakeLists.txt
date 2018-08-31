cmake_minimum_required(VERSION 2.8.3)
project(filters)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
#  libpointmatcher
)
#find_package(libpointmatcher REQUIRED)
#catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
#include_directories(${CMAKE_CURRENT_BINARY_DIR} ${Boost_INCLUDE_DIRS} ${libpointmatcher_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#add_definitions(--std=c++11)

# VoxelGrid Filter Node
add_executable(voxel_filter src/voxel_filter.cpp)
target_link_libraries(voxel_filter ${PCL_LIBRARIES})
target_link_libraries(voxel_filter ${catkin_LIBRARIES})

# PassThrough Filter Node
add_executable(passthrough_filter src/passthrough_filter.cpp)
target_link_libraries(passthrough_filter ${PCL_LIBRARIES})
target_link_libraries(passthrough_filter ${catkin_LIBRARIES})

# Statistical Outlier Removal Filter Node
add_executable(statistical_removal src/statistical_removal.cpp)
target_link_libraries(statistical_removal ${PCL_LIBRARIES})
target_link_libraries(statistical_removal ${catkin_LIBRARIES})

# Radius Outlier Removal Filter Node
add_executable(radius_removal src/statistical_removal.cpp)
target_link_libraries(radius_removal ${PCL_LIBRARIES})
target_link_libraries(radius_removal ${catkin_LIBRARIES})

# Planar Segmentation Node
add_executable(planar_segmentation src/planar_segmentation.cpp)
target_link_libraries(planar_segmentation ${PCL_LIBRARIES})
target_link_libraries(planar_segmentation ${catkin_LIBRARIES})

# Model Projection Node
add_executable(model_projection src/model_projection.cpp)
target_link_libraries(model_projection ${PCL_LIBRARIES})
target_link_libraries(model_projection ${catkin_LIBRARIES})

# Multiple Filter Node
add_executable(filter src/filter.cpp)
target_link_libraries(filter ${PCL_LIBRARIES})
target_link_libraries(filter ${catkin_LIBRARIES})

# tf Broadcaster
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

# Moving Least Squares Resampling
add_executable(resampling src/resampling.cpp)
target_link_libraries(resampling ${PCL_LIBRARIES})
target_link_libraries(resampling ${catkin_LIBRARIES})

# Iterative Closest Point Algorithm
add_executable(icp src/icp.cpp)
target_link_libraries(icp ${PCL_LIBRARIES})
target_link_libraries(icp ${catkin_LIBRARIES})

# LibPointMatcher ICP Filter
#add_executable(pointmatcher src/pointmatcher.cpp)
#target_link_libraries(pointmatcher ${PCL_LIBRARIES})
#target_link_libraries(pointmatcher ${catkin_LIBRARIES})
#target_link_libraries(pointmatcher ${libpointmatcher_LIBRARIES} ${Boost_LIBRARIES})
