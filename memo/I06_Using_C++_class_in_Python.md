# Step 6: Using a C++ class in Python

## 1. Class without NodeHandle
Because roscpp is not initialized when calling rospy.init_node. ros::NodeHandle instances cannot be used in the C++ class without generating an error. If the C++ does not make use of ros::NodeHandle, this is no issue though.

### Creating the package and writing the C++ class
Create a package and create the C++ class for which we will want to make a Python binding. This class uses ROS messages as arguments and return type. 
```
$ cd /path/to/catkin_ws/src
$ catkin_create_pkg python_bindings_tutorial rospy roscpp std_msgs
$ cd python_bindings_tutorial/include/python_bindings_tutorial
$ touch add_two_ints.h
$ rosed python_bindings_tutorial add_two_ints.h
```
The content of include/python_bindings_tutorial/add_two_ints.h will be: 
```
#ifndef PYTHON_BINDINGS_TUTORIAL_ADD_TWO_INTS_H
#define PYTHON_BINDINGS_TUTORIAL_ADD_TWO_INTS_H

#include <std_msgs/Int64.h>

namespace python_bindings_tutorial {

class AddTwoInts
{
  public:
    std_msgs::Int64 add(const std_msgs::Int64& a, const std_msgs::Int64& b);
};

} // namespace python_bindings_tutorial

#endif // PYTHON_BINDINGS_TUTORIAL_ADD_TWO_INTS_H
```
 
 Write the class implementation into
```
$ roscd python_bindings_tutorial/src
$ touch add_two_ints.cpp
$ rosed python_bindings_tutorial add_two_ints.cpp
```
 The content of src/add_two_ints.cpp will be: 
```
#include <python_bindings_tutorial/add_two_ints.h>

using namespace python_bindings_tutorial;

std_msgs::Int64 AddTwoInts::add(const std_msgs::Int64& a, const std_msgs::Int64& b)
{
  std_msgs::Int64 sum;
  sum.data = a.data + b.data;
  return sum;
}
```

### Binding C++ part
```
$ roscd python_bindings_tutorial/src
$ touch add_two_ints_wrapper.cpp
$ rosed python_bindings_tutorial add_two_ints_wrapper.cpp
```
The content of src/add_two_ints_wrapper.cpp will be: 
```
#include <boost/python.hpp>

#include <string>

#include <ros/serialization.h>
#include <std_msgs/Int64.h>

#include <python_bindings_tutorial/add_two_ints.h>

/* Read a ROS message from a serialized string.
  */
template <typename M>
M from_python(const std::string str_msg)
{
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i)
  {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

/* Write a ROS message into a serialized string.
*/
template <typename M>
std::string to_python(const M& msg)
{
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i)
  {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

class AddTwoIntsWrapper : public python_bindings_tutorial::AddTwoInts
{
  public:
    AddTwoIntsWrapper() : AddTwoInts() {}

    std::string add(const std::string& str_a, const std::string& str_b)
    {
      std_msgs::Int64 a = from_python<std_msgs::Int64>(str_a);
      std_msgs::Int64 b = from_python<std_msgs::Int64>(str_b);
      std_msgs::Int64 sum = AddTwoInts::add(a, b);

      return to_python(sum);
    }
};

BOOST_PYTHON_MODULE(_add_two_ints_wrapper_cpp)
{
  boost::python::class_<AddTwoIntsWrapper>("AddTwoIntsWrapper", boost::python::init<>())
    .def("add", &AddTwoIntsWrapper::add);
}
```

### Binding Python part
The Python wrapper translates input from Python message instances into serialized content and output from serialized content to Python message instances. The translation from Python serialized content (str) into C++ serialized content (std::string) is built in the Boost Python library. 
```
$ roscd python_bindings_tutorial/src
$ mkdir python_bindings_tutorial
$ roscd python_bindings_tutorial/src/python_bindings_tutorial
$ touch _add_two_ints_wrapper_py.py
$ rosed python_bindings_tutorial _add_two_ints_wrapper_py.py
```
The content of src/python_bindings_tutorial/_add_two_ints_wrapper_py.py will be 
```
from StringIO import StringIO

import rospy
from std_msgs.msg import Int64

from python_bindings_tutorial._add_two_ints_wrapper_cpp import AddTwoIntsWrapper


class AddTwoInts(object):
    def __init__(self):
        self._add_two_ints = AddTwoIntsWrapper()

    def _to_cpp(self, msg):
        """Return a serialized string from a ROS message

        Parameters
        ----------
        - msg: a ROS message instance.
        """
        buf = StringIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        """Return a ROS message from a serialized string

        Parameters
        ----------
        - str_msg: str, serialized message
        - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
        """
        msg = cls()
        return msg.deserialize(str_msg)

    def add(self, a, b):
        """Add two std_mgs/Int64 messages

        Return a std_msgs/Int64 instance.

        Parameters
        ----------
        - a: a std_msgs/Int64 instance.
        - b: a std_msgs/Int64 instance.
        """
        if not isinstance(a, Int64):
            rospy.ROSException('Argument 1 is not a std_msgs/Int64')
        if not isinstance(b, Int64):
            rospy.ROSException('Argument 2 is not a std_msgs/Int64')
        str_a = self._to_cpp(a)
        str_b = self._to_cpp(b)
        str_sum = self._add_two_ints.add(str_a, str_b)
        return self._from_cpp(str_sum, Int64)
```
In order to be able to import the class as python_bindings_tutorial.AddTwoInts, we import the symbols in ```__init__.py```. First, we create the file: 
```
$ roscd python_bindings_tutorial/src/python_bindings_tutorial
$ touch __init__.py
$ rosed python_bindings_tutorial __init__.py
```
The content of src/python_bindings_tutorial/__init__.py will be: 
```
from python_bindings_tutorial._add_two_ints_wrapper_py import AddTwoInts
```

### Glueing everything together
Edit the CMakeLists.txt (rosed python_bindings_tutorial CMakeLists.txt) like this: 
```
cmake_minimum_required(VERSION 2.8.3)
project(python_bindings_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roscpp_serialization
  std_msgs
)

## Both Boost.python and Python libs are required.
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES add_two_ints _add_two_ints_wrapper_cpp
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

###########
## Build ##
###########

# include Boost and Python.
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        )

## Declare a cpp library
add_library(add_two_ints src/add_two_ints.cpp)
add_library(_add_two_ints_wrapper_cpp src/add_two_ints_wrapper.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(add_two_ints ${catkin_LIBRARIES})
target_link_libraries(_add_two_ints_wrapper_cpp add_two_ints ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(_add_two_ints_wrapper_cpp PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )
```
The c++ wrapper library should be have the same name as the Python module. If the target name needs to be different for a reason, the library name can be specified with set_target_properties(_add_two_ints_wrapper_cpp PROPERTIES OUTPUT_NAME correct_library_name).

The line 
```
catkin_python_setup()
```
is used to export the Python module and is associated with the file setup.py

```
$ roscd python_bindings_tutorial
$ touch setup.py
```
The content of setup.py will be:
```
# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['python_bindings_tutorial'],
    package_dir={'': 'src'})

setup(**setup_args)
```
We then build the package with catkin_make. 

### Testing the binding
```
from std_msgs.msg import Int64
from python_bindings_tutorial import AddTwoInts
a = Int64(4)
b = Int64(2)
addtwoints = AddTwoInts()
sum = addtwoints.add(a, b)
print sum
```