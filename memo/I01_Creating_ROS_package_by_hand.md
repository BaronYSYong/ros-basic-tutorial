# Step 1. Creating a ROS package by hand

```
catkin_ws $ mkdir -p src/foobar
catkin_ws $ cd src/foobar/
```
The very first thing we'll do is add our manifest file. The package.xml file allows tools like rospack to determine information about what your package depends upon.

Inside of foobar/package.xml put the following: 
```
<package>
  <name>foobar</name>
  <version>1.2.4</version>
  <description>
  This package provides foo capability.
  </description>
  <maintainer email="foobar@foo.bar.willowgarage.com">PR-foobar</maintainer>
  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>

  <run_depend>roscpp</run_depend>
  <run_depend>std_msgs</run_depend>
</package>
```
```
$ rospack find foobar
/home/baron/bitbucket/ros-practice/catkin_ws/src/foobar
```
If ROS is set up correctly you should see something like: /home/user/ros/catkin_ws_top/src/foobar. This is how ROS finds packages behind the scenes.

Note that this package now also has dependencies on roscpp and std_msgs.

Such dependencies are used by catkin to configure packages in the right order.

Now we need the CMakeLists.txt file so that catkin_make, which uses CMake for its more powerful flexibility when building across multiple platforms, builds the package.

In foobar/CMakeLists.txt put: 
```
cmake_minimum_required(VERSION 2.8.3)
project(foobar)
find_package(catkin REQUIRED roscpp std_msgs)
catkin_package()
```