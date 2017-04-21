# Step 7. ROS Services and Parameters

## 1. ROS Services
Services allow nodes to send a request and receive a response.

## 2. rosservice
rosservice can easily attach to ROS's client/service framework with services. rosservice has many commands that can be used on topics, as shown below:

Usage: 
```
rosservice list         print information about active services
rosservice call         call the service with the provided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri
```

### rosservice list
```
$ rosservice list
/clear
/kill
/reset
/rosout/get_loggers
/rosout/set_logger_level
/spawn
/teleop_turtle/get_loggers
/teleop_turtle/set_logger_level
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/get_loggers
/turtlesim/set_logger_level
```

### rosservice type
Usage:
```
rosservice type [service]
```
Example:
```
$ rosservice type /clear
std_srvs/Empty
```
This service is empty, this means when the service call is made it takes no arguments (i.e. it sends no data when making a request and receives no data when receiving a response).

### rosservice call
Usage:
```
rosservice call [service] [args]
```
Example:
```
$ rosservice call /clear
```
it clears the background of the turtlesim_node.

```
$ rosservice type /spawn| rossrv show
float32 x
float32 y
float32 theta
string name
---
string name
```
This service lets us spawn a new turtle at a given location and orientation. The name field is optional, so let's not give our new turtle a name and let turtlesim create one for us. 
```
$ rosservice call /spawn 2 2 0.2 ""
name: turtle2
```
The service call returns with the name of the newly created turtle 

## 3. rosparam
rosparam allows you to store and manipulate data on the ROS Parameter Server. The Parameter Server can store integers, floats, boolean, dictionaries, and lists. rosparam uses the YAML markup language for syntax. In simple cases, YAML looks very natural: 1 is an integer, 1.0 is a float, one is a string, true is a boolean, [1, 2, 3] is a list of integers, and {a: b, c: d} is a dictionary. rosparam has many commands that can be used on parameters, as shown below:

Usage: 
```
rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           dump parameters to file
rosparam delete         delete parameter
rosparam list           list parameter names
```

### rosparam list
```
$ rosparam list
/background_b
/background_g
/background_r
/rosdistro
/roslaunch/uris/host_baron_endeavor_na512e__37296
/rosversion
/run_id
```

### rosparam set and rosparam get
Usage:
```
rosparam set [param_name]
rosparam get [param_name]
```
Here will change the red channel of the background color: 
```
$ rosparam set /background_r 150
```
This changes the parameter value, now we have to call the clear service for the parameter change to take effect: 
```
$ rosservice call /clear
```
Get the value of the green background channel: 
```
$ rosparam get /background_g
86
```
Use rosparam get / to show us the contents of the entire Parameter Server. 
```
$ rosparam get /
background_b: 255
background_g: 86
background_r: 150
rosdistro: 'indigo

  '
roslaunch:
  uris: {host_baron_endeavor_na512e__37296: 'http://baron-Endeavor-NA512E:37296/'}
rosversion: '1.11.21

  '
run_id: 4fb5e6a8-23e1-11e7-b488-a402b9d9a954
```

### rosparam dump and rosparam load
Usage:
```
rosparam dump [file_name] [namespace]
rosparam load [file_name] [namespace]
```
Here we write all the parameters to the file params.yaml 
```
$ rosparam dump params.yaml
```
Load these yaml files into new namespaces, e.g. copy: 
```
$ rosparam load params.yaml copy
$ rosparam get /copy/background_b
255
```