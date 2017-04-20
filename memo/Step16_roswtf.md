# Step 16. roswtf

## 1. Check installation
roswtf examines your system to try and find problems. Let's try it out:
```
$ roscd
$ roswtf
Loaded plugin tf.tfwtf
No package or stack in context
================================================================================
Static checks summary:

No errors or warnings
================================================================================

ROS Master does not appear to be running.
Online graph checks will not be run.
ROS_MASTER_URI is [http://localhost:11311]
```

## 2. Trying it online
For this next step, we want a Master to be up, so go ahead and start a roscore.

```
$ roscd
$ roswtf
Loaded plugin tf.tfwtf
No package or stack in context
================================================================================
Static checks summary:

No errors or warnings
================================================================================
Beginning tests of your ROS graph. These may take awhile...
analyzing graph...
... done analyzing graph
running graph rules...
... done running graph rules

Online checks summary:

Found 1 warning(s).
Warnings are things that may be just fine, but are sometimes at fault

WARNING The following node subscriptions are unconnected:
 * /rosout:
   * /rosout
```
roswtf did some online examination of your graph now that your roscore is running. Depending on how many ROS nodes you have running, this can take a long time to complete. As you can see, this time it produced a warning: 
```
WARNING The following node subscriptions are unconnected:
 * /rosout:
   * /rosout
```
roswtf is warning you that the rosout node is subscribed to a topic that no one is publishing to. In this case, this is expected because nothing else is running, so we can ignore it.

## 3. Errors
roswtf will warn you about things that look suspicious but may be normal in your system. It can also report errors for problems that it knows are wrong.

For this part, we are going to set your ROS_PACKAGE_PATH to a bad value. We're also going to stop our roscore to simplify the output that you see. 
```
$ roscd
$ ROS_PACKAGE_PATH=bad:$ROS_PACKAGE_PATH roswtf
Loaded plugin tf.tfwtf
No package or stack in context
================================================================================
Static checks summary:

Found 1 error(s).

ERROR Not all paths in ROS_PACKAGE_PATH [bad:/home/baron/bitbucket/ros-practice/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks] point to an existing directory: 
 * bad

================================================================================

ROS Master does not appear to be running.
Online graph checks will not be run.
ROS_MASTER_URI is [http://localhost:11311]
```
As you can see, roswtf now gives us an error about the ROS_PACKAGE_PATH setting.

There are many other types of problems that roswtf can find. If you find yourself stumped by a build or communication issue, try running it and seeing if it can point you in the right direction. 