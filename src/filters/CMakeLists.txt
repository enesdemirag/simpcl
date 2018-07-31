cmake_minimum_required(VERSION 2.8.3)
project(filters)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# VoxelGrid Filter Node
add_executable(voxel_filter src/voxel_filter.cpp)
target_link_libraries(voxel_filter ${PCL_LIBRARIES})
target_link_libraries(voxel_filter ${catkin_LIBRARIES})

# PassThrough Filter Node
add_executable (passthrough_filter src/passthrough_filter.cpp)
target_link_libraries (passthrough_filter ${PCL_LIBRARIES})
target_link_libraries(passthrough_filter ${catkin_LIBRARIES})

# Statistical Outlier Removal Filter Node
add_executable (statistical_removal src/statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})
target_link_libraries(statistical_removal ${catkin_LIBRARIES})

# Planar Segmentation Node
add_executable (planar_segmentation src/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})
target_link_libraries(planar_segmentation ${catkin_LIBRARIES})

# Model Projection Node
add_executable (model_projection src/model_projection.cpp)
target_link_libraries (model_projection ${PCL_LIBRARIES})
target_link_libraries(model_projection ${catkin_LIBRARIES})
