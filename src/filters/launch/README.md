# Launch files for filters #

Parameters can easily be changed without the need to recompile

#### Example
-------
``` <param name="parameter_name" type="parameter_type" value="parameter_value"/> ```

#### Install OctoMap

```
$ sudo apt-get install build-essential cmake doxygen libqt4-dev libqt4-opengl-dev libqglviewer-dev-qt4
$ sudo apt-get install ros-kinetic-octomap ros-kinetic-octomap-mapping
$ rosdep install octomap_mapping
$ rosmake octomap_mapping
```

* There should be ```octomap_mapping.launch``` file under ```/opt/ros/kinetic/share/octomap_server/launch``` now
