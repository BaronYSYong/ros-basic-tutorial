# Step 2. Navigating the ROS Filesystem

## rospack
rospack allows you to get information about packages

Usage  
 ```
$ rospack find [package_name]
 ```
 
Example  
```
$ rospack find roscpp
/opt/ros/indigo/share/roscpp
```

## roscd
roscd is part of the rosbash suite. It allows you to change directory (cd) directly to a package or a stack.

Usage  
 ```
$ roscd [locationname[/subdir]]
 ```
 
Example  
```
$ roscd roscpp
$ pwd
/opt/ros/indigo/share/roscpp
```
```
$ roscd roscpp/cmake
$ pwd
/opt/ros/indigo/share/roscpp/cmake
```
roscd log will take you to the folder where ROS stores log files
```
$ roscd log
```

## rosls
rosls is part of the rosbash suite. It allows you to ls directly in a package by name rather than by absolute path.   

Usage 
```
$ rosls [locationname[/subdir]]
```
Example
```
$ rosls roscpp_tutorials
cmake  launch  package.xml  srv
```

