[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://enesdemirag.mit-license.org)

# Point Cloud Filtering <img align="right" src="https://raw.githubusercontent.com/enesdemirag/enesdemirag.github.io/master/images/simpcl/logo.png?token=AGJG3QEMBM7TZCITPL35TPC6DXC7W">

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
![alt text](https://raw.githubusercontent.com/enesdemirag/enesdemirag.github.io/master/images/simpcl/otopark_raw.png?token=AGJG3QBSUWFLXA4Y52JZKX26DXDBW "Raw Point Cloud Data")
### Downsampling with "Voxel Grid Filter"
![alt text](https://raw.githubusercontent.com/enesdemirag/enesdemirag.github.io/master/images/simpcl/otopark_voxel.png?token=AGJG3QDX4LAXA3AYJPRMVR26DXDBY "Voxel Grid Filter")
### Getting points between 0.5m to 18m with "Passthrough Filter"
![alt text](https://raw.githubusercontent.com/enesdemirag/enesdemirag.github.io/master/images/simpcl/otopark_passthrough.png?token=AGJG3QGQECK2AXMB54F6QIS6DXDB2 "Voxel Grid Filter")
### Removing noise with "Statistical Outlier Removal Filter"
![alt text](https://raw.githubusercontent.com/enesdemirag/enesdemirag.github.io/master/images/simpcl/otopark_statistical.png?token=AGJG3QBRTUUKMNNHCKGCZH26DXDFC "Statistical Outlier Removal Filter")
### 3D mapping with OctoMap
![alt text](https://github.com/enesdemirag/simpcl/raw/internship/images/octomap_otopark.gif "Mapping using Odometry and Point Cloud data simultaneously")
### Mapping Graph
![alt text](https://raw.githubusercontent.com/enesdemirag/enesdemirag.github.io/master/images/simpcl/mapping_graph.png?token=AGJG3QHZ4GBA2I7EOBQ36I26DXDFI "from rqt_graph")
### Filtering Graph
![alt text](https://raw.githubusercontent.com/enesdemirag/enesdemirag.github.io/master/images/simpcl/filtering_graph.png?token=AGJG3QFKWYE3VLMM76HI4YS6DXDFK "from rqt_graph")
---
# How to Use #

Get this repository as a catkin workspace
```
$ git clone https://github.com/enesdemirag/simpcl.git catkin_ws
```
Build package
```
$ cd catkin_ws
$ catkin build
```
Launch files are under the *[src/filter/launch directory](https://github.com/enesdemirag/simpcl/tree/internship/src/filters/launch)*
Parameters can easily be changed without the need to recompile.
```
$ roslaunch filters mapping.launch
```

#### Example Parameter

```
<param name="parameter_name" type="parameter_type" value="parameter_value"/>
```

## Author

* **Enes Demirağ** <ensdmrg@gmail.com> - *[LinkedIn](https://www.linkedin.com/in/enesdemirag/)* - *[Website](https://enesdemirag.github.io)*

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

### Work in progress
* ~~Removing ground using segmentation methods~~
* ~~Filtering with ICP Algorithm~~
* Real-time mapping

#### Planar Segmentation
![alt text](https://github.com/enesdemirag/simpcl/raw/internship/images/planar_segmentation.gif "axis color -> remained parts")
---
> `Note:` Difference of Normals Based Segmentation (don_segmentation.cpp)
and Progressive Morphological Segmentation (morphological_segmentation.cpp) nodes are not working properly for now. Additionally [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) library used in pointmatcher node instead of PCL, but it is also not working because of wrong initial translation and rotation sizes.

#### ICP
![alt text](https://github.com/enesdemirag/simpcl/raw/internship/images/icp.gif "mapping after ICP filter")
---
* Results: ([octovis](http://wiki.ros.org/octovis) required)
    * [with 5 meter sight](#)
    * [with 10 meter sight](#)
