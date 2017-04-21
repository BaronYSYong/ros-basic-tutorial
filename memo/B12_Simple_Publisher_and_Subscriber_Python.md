# Step 11. Simple_Publisher_and_Subscriber_Python

## 1. Publisher Node
```
$ roscd beginner_tutorials
$ mkdir scripts
$ cd scripts
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
$ chmod +x talker.py
```

### code explanation
```
#!/usr/bin/env python
```
Every Python ROS Node will have this declaration at the top. The first line makes sure your script is executed as a Python script. 
```
import rospy
from std_msgs.msg import String
```
You need to import rospy if you are writing a ROS Node. The std_msgs.msg import is so that we can reuse the std_msgs/String message type (a simple string container) for publishing. 
```
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
```
This section of code defines the talker's interface to the rest of ROS. pub = rospy.Publisher("chatter", String, queue_size=10) declares that your node is publishing to the chatter topic using the message type String. String here is actually the class std_msgs.msg.String. The queue_size argument is New in ROS hydro and limits the amount of queued messages if any subscriber is not receiving the them fast enough. In older ROS distributions just omit the argument.

The next line, rospy.init_node(NAME), is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS Master. In this case, your node will take on the name talker. NOTE: the name must be a base name, i.e. it cannot contain any slashes "/". 
```
    rate = rospy.Rate(10) # 10hz
```
This line creates a Rate object rate. With the help of its method sleep(), it offers a convenient way for looping at the desired rate. With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!) 
```
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
```
This loop is a fairly standard rospy construct: checking the rospy.is_shutdown() flag and then doing work. You have to check is_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise). In this case, the "work" is a call to pub.publish(hello_str) that publishes a string to our chatter topic. The loop calls r.sleep(), which sleeps just long enough to maintain the desired rate through the loop.

(You may also run across rospy.sleep() which is similar to time.sleep() except that it works with simulated time as well (see Clock).)

This loop also calls rospy.loginfo(str), which performs triple-duty: the messages get printed to screen, it gets written to the Node's log file, and it gets written to rosout. rosout is a handy for debugging: you can pull up messages using rqt_console instead of having to find the console window with your Node's output.

std_msgs.msg.String is a very simple message type, so you may be wondering what it looks like to publish more complicated types. The general rule of thumb is that constructor args are in the same order as in the .msg file. You can also pass in no arguments and initialize the fields directly, e.g. 
```
msg = String()
msg.data = str
```
or you can initialize some of the fields and leave the rest with default values:
```
String(data=str)
```
You may be wondering about the last little bit:
```
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
In addition to the standard Python __main__ check, this catches a rospy.ROSInterruptException exception, which can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when Ctrl-C is pressed or your Node is otherwise shutdown. The reason this exception is raised is so that you don't accidentally continue executing code after the sleep(). 

## 2. Subscriber Node
```
$ roscd beginner_tutorials/scripts/
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
```
Don't forget to make the node executable: 
```
$ chmod +x listener.py
```

### code explanation
The code for listener.py is similar to talker.py, except we've introduced a new callback-based mechanism for subscribing to messages. 
```
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
```
This declares that your node subscribes to the chatter topic which is of type std_msgs.msgs.String. When new messages are received, callback is invoked with the message as the first argument.

We also changed up the call to rospy.init_node() somewhat. We've added the anonymous=True keyword argument. ROS requires that each node have a unique name. If a node with the same name comes up, it bumps the previous one. This is so that malfunctioning nodes can easily be kicked off the network. The anonymous=True flag tells rospy to generate a unique name for the node so that you can have multiple listener.py nodes run easily.

The final addition, rospy.spin() simply keeps your node from exiting until the node has been shutdown. Unlike roscpp, rospy.spin() does not affect the subscriber callback functions, as those have their own threads. 

## 3. Building nodes
```
$ cd ~/catkin_ws
$ catkin_make
```

## 4. Examining the simple publisher and subscriber
Make sure that a roscore is up and running: 
```
$ roscore
```
Try run the command
```
$ rosrun beginner_tutorials talker.py
```
In a new terminal
```
$ rosrun beginner_tutorials listener.py
```
