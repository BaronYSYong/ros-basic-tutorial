# Step 4. Building ROS Package

## 1. catkin_make
catkin_make combines the calls to cmake and make in the standard CMake workflow

```
# In a catkin workspace
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```

The commands below will build any catkin projects found in the src folder. 
```
# In a catkin workspace
$ catkin_make
$ catkin_make install  # (optionally)
```

If your source code is in a different place, say my_src then you would call catkin_make like this:
```
# In a catkin workspace
$ catkin_make --source my_src
$ catkin_make install --source my_src  # (optionally)
```

## 2. Building own package
```
$ cd ~/catkin_ws/
$ ls src
beginner_tutorials  CMakeLists.txt
$ catkin_make
```
```
$ ls
build  devel  src
```
* build folder is the default location of the build space and is where cmake and make are called to configure and build your packages. 
* devel folder is the default location of the devel space, which is where your executables and libraries go before you install your packages. 