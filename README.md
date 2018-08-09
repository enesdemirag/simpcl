[![Build Status](http://img.shields.io/travis/badges/badgerbadgerbadger.svg?style=flat-square)](https://travis-ci.org/badges/badgerbadgerbadger) [![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://enesdemirag.mit-license.org)

# Point Cloud Filtering # <img align="right" src="https://github.com/enesdemirag/zed_filtering/raw/master/images/logo.png">

My internship project in *[Ravinspect](http://www.ravinspect.com)*. Filtering point cloud data coming from ZED camera in real-time.

This project can work properly with all devices which produce point cloud data. (Stereo Cameras, RGB-D Cameras, LiDARs etc.) *[Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl)* and *[OctoMap](https://github.com/OctoMap/octomap_mapping)* used on ROS platform.

# Installation #

#### Install ROS
Check out the *[official website](http://wiki.ros.org/kinetic/Installation)*

#### Install Point Cloud Library
Download the stable release from *[here](https://github.com/PointCloudLibrary/pcl/releases)* and follow these *[instructions](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php)*

#### Install OctoMap

```
$ sudo apt-get install build-essential cmake doxygen libqt4-dev libqt4-opengl-dev libqglviewer-dev-qt4
$ sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping
$ rosdep install octomap_mapping
$ rosmake octomap_mapping
```

* There should be ```octomap_mapping.launch``` file under ```/opt/ros/kinetic/share/octomap_server/launch``` now

# Filtering Results #

### Raw Point Cloud
![alt text](https://github.com/enesdemirag/zed_filtering/blob/master/images/otopark_raw.png "Raw Point Cloud Data")
### Downsampling with "Voxel Grid Filter"
![alt text](https://github.com/enesdemirag/zed_filtering/blob/master/images/otopark_voxel.png "Voxel Grid Filter")
### Getting points between 0.5m to 18m with "Passthrough Filter"
![alt text](https://github.com/enesdemirag/zed_filtering/blob/master/images/otopark_passthrough.png "Voxel Grid Filter")
### Removing noise with "Statistical Outlier Removal Filter"
![alt text](https://github.com/enesdemirag/zed_filtering/blob/master/images/otopark_statistical.png "Statistical Outlier Removal Filter")
### 3D mapping with OctoMap
![alt text](https://github.com/enesdemirag/zed_filtering/blob/master/images/octomap_otopark.gif "Mapping using Odometry and Point Cloud data simultaneously")
### Mapping Graph
![alt text](https://github.com/enesdemirag/zed_filtering/blob/master/images/mapping_graph.png "from rqt_graph")
### Filtering Graph
![alt text](https://github.com/enesdemirag/zed_filtering/blob/master/images/filtering_graph.png "from rqt_graph")

# Launch files for filters #

Parameters can easily be changed without the need to recompile

#### Example Parameter

``` <param name="parameter_name" type="parameter_type" value="parameter_value"/> ```

# Documents about Point Cloud Processing #

* **An Efficient Probabilistic 3D Mapping Framework Based on Octrees** - *[Article](http://web.itu.edu.tr/demirag16/media/docs/OctoMap.pdf)*

* **3D Mapping with OctoMap** - *[Presentation](http://www2.informatik.uni-freiburg.de/~hornunga/pub/hornung13roscon.pdf)*

* **Fast Resampling of 3D Point Clouds via Graphs.** - *[Article](http://web.itu.edu.tr/demirag16/media/docs/FastResamplingof3DPointCloudsviaGraphs.pdf)*

* **Modeling the World Around Us** - *[Presentation](http://web.itu.edu.tr/demirag16/media/docs/MappingOverview.pdf)*

* **Point Cloud Library: Filtering** - *[Presentation](http://web.itu.edu.tr/demirag16/media/docs/Filtering.pdf)*

* **Pairwise, Rigid Registration The ICP Algorithm and Its Variants** - *[Presentation](Pairwise-RigidRegistration.pdf)*

* **Point Cloud Segmentation and Classification Algorithms** - *[Report](http://web.itu.edu.tr/demirag16/media/docs/PointCloudSegmentationAndClassificationAlgorithms.pdf)*

* **Progressive Morphological Filter for Removing Nonground Measurements** - *[Article](http://web.itu.edu.tr/demirag16/media/docs/ProgressiveMorphologicalFilter.pdf)*

* **Difference of Normals as a Multi-Scale Operator in Unorganized Point Clouds** - *[Article](http://web.itu.edu.tr/demirag16/media/docs/DoNSegmentation.pdf)*

* **Plane Detection and Segmentation For DARPA Robotics Challange** - *[Article](http://web.itu.edu.tr/demirag16/media/docs/PlaneDetectionandSegmentation.pdf)*

* **Automated Extraction of Urban Road Facilities** - *[Article](http://web.itu.edu.tr/demirag16/media/docs/StudiesonGroundRemoval.pdf)*

## Author

* **Enes Demirağ** <demirag16@itu.edu.tr> - *[LinkedIn](https://www.linkedin.com/in/enesdemirag/)*

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

### Work in progress
* Removing ground using segmentation methods

#### Planar Segmentation
![alt text](https://github.com/enesdemirag/zed_filtering/blob/master/images/planar_segmentation.gif "red -> removed parts")
---
> `Note:` Difference of Normals Based Segmentation (don_segmentation.cpp)
and Progressive Morphological Segmentation (morphological_segmentation.cpp) nodes are not working properly for now.
