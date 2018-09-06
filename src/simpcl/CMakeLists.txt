cmake_minimum_required(VERSION 2.8.3)
project(simpcl)

add_definitions(-std=c++11)
add_definitions(${PCL_DEFINITIONS})

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    pcl_ros
    tf
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# ------------------------------------------------------- #

# All in One Node
add_executable(all_in_one src/all_in_one.cpp)
target_link_libraries(all_in_one ${PCL_LIBRARIES})
target_link_libraries(all_in_one ${catkin_LIBRARIES})

# tf Broadcaster
add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

# ------------------------------------------------------- #

# VoxelGrid Filter Node
add_executable(voxel_filter src/filters/voxel_filter.cpp)
target_link_libraries(voxel_filter ${PCL_LIBRARIES})
target_link_libraries(voxel_filter ${catkin_LIBRARIES})

# PassThrough Filter Node
add_executable(passthrough_filter src/filters/passthrough_filter.cpp)
target_link_libraries(passthrough_filter ${PCL_LIBRARIES})
target_link_libraries(passthrough_filter ${catkin_LIBRARIES})

# Statistical Outlier Removal Filter Node
add_executable(statistical_removal src/filters/statistical_removal.cpp)
target_link_libraries(statistical_removal ${PCL_LIBRARIES})
target_link_libraries(statistical_removal ${catkin_LIBRARIES})

# Radius Outlier Removal Filter Node
add_executable(radius_removal src/filters/statistical_removal.cpp)
target_link_libraries(radius_removal ${PCL_LIBRARIES})
target_link_libraries(radius_removal ${catkin_LIBRARIES})

# Planar Segmentation Node
add_executable(planar_segmentation src/filters/planar_segmentation.cpp)
target_link_libraries(planar_segmentation ${PCL_LIBRARIES})
target_link_libraries(planar_segmentation ${catkin_LIBRARIES})

# Model Projection Node
add_executable(model_projection src/filters/model_projection.cpp)
target_link_libraries(model_projection ${PCL_LIBRARIES})
target_link_libraries(model_projection ${catkin_LIBRARIES})

# Multiple Filter Node
add_executable(voxel_pass_removal src/filters/voxel_pass_removal.cpp)
target_link_libraries(voxel_pass_removal ${PCL_LIBRARIES})
target_link_libraries(voxel_pass_removal ${catkin_LIBRARIES})

# Iterative Closest Point Algorithm
add_executable(icp src/filters/icp.cpp)
target_link_libraries(icp ${PCL_LIBRARIES})
target_link_libraries(icp ${catkin_LIBRARIES})
