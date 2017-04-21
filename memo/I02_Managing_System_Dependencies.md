# Step 2. Managing System dependencies

## 1. System Dependencies
ROS packages sometimes require external libraries and tools that must be provided by the operating system. These required libraries and tools are commonly referred to as system dependencies. In some cases these system dependencies are not installed by default. ROS provides a simple tool, rosdep, that is used to download and install system dependencies
```
$ roscd turtlesim
$ cat package.xml
```

### rosdep
rosdep is a tool you can use to install system dependencies required by ROS packages. 
Usage:
```
rosdep install [package]
```
Download and install the system dependencies for turtlesim: 
```
$ rosdep install turtlesim
```