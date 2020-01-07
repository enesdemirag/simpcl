[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://enesdemirag.mit-license.org)

# Point Cloud Filtering <img align="right" src="https://raw.githubusercontent.com/enesdemirag/point-cloud-filters/internship/images/logo.png">

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

There should be ```octomap_mapping.launch``` file under ```/opt/ros/kinetic/share/octomap_server/launch``` now

# Filtering Results #

### Raw Point Cloud
![alt text](https://github.com/enesdemirag/point-cloud-filters/blob/internship/images/otopark_raw.png "Raw Point Cloud Data")
### Downsampling with "Voxel Grid Filter"
![alt text](https://github.com/enesdemirag/point-cloud-filters/blob/internship/images/otopark_voxel.png "Voxel Grid Filter")
### Getting points between 0.5m to 18m with "Passthrough Filter"
![alt text](https://github.com/enesdemirag/point-cloud-filters/blob/internship/images/otopark_passthrough.png "Voxel Grid Filter")
### Removing noise with "Statistical Outlier Removal Filter"
![alt text](https://github.com/enesdemirag/point-cloud-filters/blob/internship/images/otopark_statistical.png "Statistical Outlier Removal Filter")
### 3D mapping with OctoMap
![alt text](https://github.com/enesdemirag/point-cloud-filters/blob/internship/images/octomap_otopark.gif "Mapping using Odometry and Point Cloud data simultaneously")
### Mapping Graph
![alt text](https://github.com/enesdemirag/point-cloud-filters/blob/internship/images/mapping_graph.png "from rqt_graph")
### Filtering Graph
![alt text](https://github.com/enesdemirag/point-cloud-filters/blob/internship/images/filtering_graph.png "from rqt_graph")
---
# How to Use #

Get this repository as a catkin workspace
```
$ git clone https://github.com/enesdemirag/point-cloud-filters.git catkin_ws
```
Build package
```
$ cd catkin_ws
$ catkin build
```
Launch files are under the *[src/filter/launch directory](https://github.com/enesdemirag/point-cloud-filters/tree/internship/src/filters/launch)*
Parameters can easily be changed without the need to recompile.
```
$ roslaunch filters mapping.launch
```

#### Example Parameter

```
<param name="parameter_name" type="parameter_type" value="parameter_value"/>
```

# Documents about Point Cloud Processing #

* **Introduction to PCL** - *[Presentation](http://web.itu.edu.tr/demirag16/media/docs/PCLIntro.pdf)*

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

* **ICP Algorithm and Its Variants** - *[Presentation](http://web.itu.edu.tr/demirag16/media/docs/Pairwise-RigidRegistration.pdf)*

## Author

* **Enes Demirağ** <ensdmrg@gmail.com> - *[LinkedIn](https://www.linkedin.com/in/enesdemirag/)*

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

### Work in progress
* ~~Removing ground using segmentation methods~~
* ~~Filtering with ICP Algorithm~~
* Real-time mapping

#### Planar Segmentation
![alt text](https://github.com/enesdemirag/point-cloud-filters/blob/internship/images/planar_segmentation.gif "axis color -> remained parts")
---
> `Note:` Difference of Normals Based Segmentation (don_segmentation.cpp)
and Progressive Morphological Segmentation (morphological_segmentation.cpp) nodes are not working properly for now. Additionally [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) library used in pointmatcher node instead of PCL, but it is also not working because of wrong initial translation and rotation sizes.

#### ICP
![alt text](https://github.com/enesdemirag/point-cloud-filters/blob/internship/images/icp.gif "mapping after ICP filter")
---
* Results: ([octovis](http://wiki.ros.org/octovis) required)
    * [with 5 meter sight](http://web.itu.edu.tr/demirag16/media/5m.bt)
    * [with 10 meter sight](http://web.itu.edu.tr/demirag16/media/10m.bt)
