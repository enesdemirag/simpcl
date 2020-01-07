[![License](http://img.shields.io/:license-mit-blue.svg?style=flat-square)](http://enesdemirag.mit-license.org)

# Simple Point Cloud Filtering Package
<img align="right" src="https://raw.githubusercontent.com/enesdemirag/enesdemirag.github.io/master/images/simpcl.png?token=AGJG3QDLUDNYQO4JC66J7E26DWZRS" height="165" width="360">

This package can work properly with all devices which produce point cloud data. (Stereo Cameras, RGB-D Cameras, LiDARs etc.) *[Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl)* and *[OctoMap](https://github.com/OctoMap/octomap_mapping)* used on ROS platform.

## Requirements

* #### Install Ubuntu 16.04
    * Follow *[these steps](https://tutorials.ubuntu.com/tutorial/tutorial-install-ubuntu-desktop)* to install Ubuntu 16.04 operating system

* #### Install ROS Kinetic
    * Check out the *[official website](http://wiki.ros.org/kinetic/Installation)*

* #### Install Point Cloud Library
    * Download the stable release from *[here](https://github.com/PointCloudLibrary/pcl/releases)* and follow these *[instructions](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php)*

* #### Install OctoMap
    ```
    $ sudo apt-get install build-essential cmake doxygen libqt4-dev libqt4-opengl-dev libqglviewer-dev-qt4
    $ sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping
    $ rosdep install octomap_mapping
    $ rosmake octomap_mapping
    ```

    There should be ```octomap_mapping.launch``` file under ```/opt/ros/kinetic/share/octomap_server/launch``` now

## Installation

* Get this repository as a catkin workspace
    ```
    $ git clone https://github.com/enesdemirag/simpcl.git catkin_ws
    ```

* Build package
    ```
    $ cd catkin_ws
    $ catkin build
    ```

* Or move *[src/simpcl/](https://github.com/enesdemirag/simpcl/tree/simpcl/src/simpcl)* package under your workspace.

* Source catkin workspace
    ```
    $ source devel/setup.bash
    ```

* Launch files are under the *[src/simpcl/launch](https://github.com/enesdemirag/simpcl/tree/simpcl/src/simpcl/launch)* directory. Parameters can easily be changed without the need to recompile.

## Usage

```
$ roslaunch simpcl mapping.launch
```

![alt text](https://github.com/enesdemirag/simpcl/raw/internship/images/icp.gif "Mapping with ICP")

## Author

* **Enes Demirağ** <ensdmrg@gmail.com> - *[LinkedIn](https://www.linkedin.com/in/enesdemirag/)* *[Website](https://enesdemirag.github.io)*

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contributing

Interested in contributing to my repo? Fork, and open a pull request. I'll try to merge as soon as possible. If you found any mistake or you have advice, feel free to open an issue.
