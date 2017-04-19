# Step 11. Simple_Publisher_and_Subscriber_C++

## 1. Publisher Node
Create talker.cpp file within the beginner_tutorials package
```
$ roscd beginner_tutorials
$ touch src/talker.cpp
```

### code explanation
```
#include "ros/ros.h"
```
ros/ros.h is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system.
```
#include "std_msgs/String.h"
```
This includes the std_msgs/String message, which resides in the std_msgs package. This is a header generated automatically from the String.msg file in that package.
```
ros::init(argc, argv, "talker");
```
Initialize ROS. This allows ROS to do name remapping through the command line -- not important for now. This is also where we specify the name of our node. Node names must be unique in a running system.

The name used here must be a base name, ie. it cannot have a / in it. 
```
ros::NodeHandle n;
```
Create a handle to this process' node. The first NodeHandle created will actually do the initialization of the node, and the last one destructed will cleanup any resources the node was using. 
```
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
```
Tell the master that we are going to be publishing a message of type std_msgs/String on the topic chatter. This lets the master tell any nodes listening on chatter that we are going to publish data on that topic. The second argument is the size of our publishing queue. In this case if we are publishing too quickly it will buffer up a maximum of 1000 messages before beginning to throw away old ones.

NodeHandle::advertise() returns a ros::Publisher object, which serves two purposes: 1) it contains a publish() method that lets you publish messages onto the topic it was created with, and 2) when it goes out of scope, it will automatically unadvertise. 
```
ros::Rate loop_rate(10);
```
A ros::Rate object allows you to specify a frequency that you would like to loop at. It will keep track of how long it has been since the last call to Rate::sleep(), and sleep for the correct amount of time.

In this case we tell it we want to run at 10Hz. 
```
  int count = 0;
  while (ros::ok())
  {
```
By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause ros::ok() to return false if that happens.

ros::ok() will return false if:

* a SIGINT is received (Ctrl-C)
* we have been kicked off the network by another node with the same name
* ros::shutdown() has been called by another part of the application.
* all ros::NodeHandles have been destroyed 

Once ros::ok() returns false, all ROS calls will fail. 
```
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
```
We broadcast a message on ROS using a message-adapted class, generally generated from a msg file. More complicated datatypes are possible, but for now we're going to use the standard String message, which has one member: "data". 
```
    chatter_pub.publish(msg);
```
Now we actually broadcast the message to anyone who is connected.
```
    ROS_INFO("%s", msg.data.c_str());
```
ROS_INFO and friends are our replacement for printf/cout. See the rosconsole documentation for more information. 
```
    ros::spinOnce();
```
Calling ros::spinOnce() here is not necessary for this simple program, because we are not receiving any callbacks. However, if you were to add a subscription into this application, and did not have ros::spinOnce() here, your callbacks would never get called. So, add it for good measure.
```
    loop_rate.sleep();
```
Now we use the ros::Rate object to sleep for the time remaining to let us hit our 10Hz publish rate.

Here's the condensed version of what's going on:

* Initialize the ROS system
* Advertise that we are going to be publishing std_msgs/String messages on the chatter topic to the master
* Loop while publishing messages to chatter 10 times a second 

Now we need to write a node to receive the messsages. 

## 2. Subscriber Node
Create listener.cpp file within the beginner_tutorials package
```
$ roscd beginner_tutorials
$ touch src/listener.cpp
```

### code explanation
```
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
```
This is the callback function that will get called when a new message has arrived on the chatter topic. The message is passed in a boost shared_ptr, which means you can store it off if you want, without worrying about it getting deleted underneath you, and without copying the underlying data. 
```
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
```
Subscribe to the chatter topic with the master. ROS will call the chatterCallback() function whenever a new message arrives. The 2nd argument is the queue size, in case we are not able to process messages fast enough. In this case, if the queue reaches 1000 messages, we will start throwing away old messages as new ones arrive.

NodeHandle::subscribe() returns a ros::Subscriber object, that you must hold on to until you want to unsubscribe. When the Subscriber object is destructed, it will automatically unsubscribe from the chatter topic.

There are versions of the NodeHandle::subscribe() function which allow you to specify a class member function, or even anything callable by a Boost.Function object. The roscpp overview contains more information. 
```
  ros::spin();
```
ros::spin() enters a loop, calling message callbacks as fast as possible. Don't worry though, if there's nothing for it to do it won't use much CPU. ros::spin() will exit once ros::ok() returns false, which means ros::shutdown() has been called, either by the default Ctrl-C handler, the master telling us to shutdown, or it being called manually.

There are other ways of pumping callbacks, but we won't worry about those here. The roscpp_tutorials package has some demo applications which demonstrate this. The roscpp overview also contains more information.

Again, here's a condensed version of what's going on:

* Initialize the ROS system
* Subscribe to the chatter topic
* Spin, waiting for messages to arrive
* When a message arrives, the chatterCallback() function is called 

## 3. Building nodes
CMakeLists.txt
```
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```
This will create two executables, talker and listener, which by default will go into package directory of your devel space, located by default at ~/catkin_ws/devel/lib/{package name}.

Note that you have to add dependencies for the executable targets to message generation targets: 
