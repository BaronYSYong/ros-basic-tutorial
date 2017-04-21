# Step 10. Creating a ROS msg and srv

## 1. Introduction to msg and srv
* msg: msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.

* srv: an srv file describes a service. It is composed of two parts: a request and a response.

msg files are stored in the msg directory of a package, and srv files are stored in the srv directory.

msgs are just simple text files with a field type and field name per line. The field types you can use are:

*    int8, int16, int32, int64 (plus uint*)
*    float32, float64
*    string
*    time, duration
*    other msg files
*    variable-length array[] and fixed-length array[C] 

Here is an example of a msg that uses a Header, a string primitive, and two other msgs : 
```
  Header header
  string child_frame_id
  geometry_msgs/PoseWithCovariance pose
  geometry_msgs/TwistWithCovariance twist
```
srv files are just like msg files, except they contain two parts: a request and a response. The two parts are separated by a '---' line. Here is an example of a srv file: 
```
int64 A
int64 B
---
int64 Sum
```
In the above example, A and B are the request, and Sum is the response.


## 2. msg
```
$ roscd beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```
Open package.xml, and make sure these two lines are in it and uncommented: 
```
  <build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>
```
Note that at build time, we need "message_generation", while at runtime, we only need "message_runtime". 

Open CMakeLists.txt and add the message_generation dependency to the find_package call which already exists in your CMakeLists.txt so that you can generate messages. You can do this by simply adding message_generation to the list of COMPONENTS such that it looks like this: 
```
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```
```
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```
```
add_message_files(
  FILES
  Num.msg
)
```
```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

## 3. rosmsg
Usage:
```
$ rosmsg show [message type]
```
Example:
```
$ rosmsg show beginner_tutorials/Num
int64 num
```
```
$ rosmsg show Num
[beginner_tutorials/Num]:
int64 num
```

## 4. srv
```
$ roscd beginner_tutorials
$ mkdir srv
```
### roscp
roscp is a useful commandline tool for copying files from one package to another.
Usage:
```
$ roscp [package_name] [file_to_copy_path] [copy_path]
```
Example:
```
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

### rossrv
Make sure that ROS can see it using the rossrv show command. 
Usage:
```
$ rossrv show <service type>
```
Example:
```
$ rossrv show beginner_tutorials/AddTwoInts
int64 a
int64 b
---
int64 sum
```
```
$ rossrv show AddTwoInts
[beginner_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum

[rospy_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum
```

## 5. Common step for msg and srv
```
$ roscd beginner_tutorials
$ cd ../..
$ catkin_make install
$ cd -
```
