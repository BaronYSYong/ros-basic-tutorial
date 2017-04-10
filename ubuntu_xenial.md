# Ubuntu 16.04 / ROS Kinetic

## Problems
* Problem1 
```
ERROR: cannot download default sources list from:
https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
Website may be down.
```

* Solution
```
$ sudo c_rehash /etc/ssl/certs
```