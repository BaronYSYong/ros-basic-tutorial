# Nextage Tutorial

## Webpage
* http://wiki.ros.org/rtmros_nextage/Tutorials
* http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_RTM

## Command
* robot.setTargetPose('rarm', [0.32, -0.18, 0.06], [-3, -1.5, 3.0], tm=3)
* robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=0.01, tm=3)
* robot.getCurrentPosition('RARM_JOINT5')
* robot.getCurrentRPY('RARM_JOINT5')
* setJointAngle(self, jname, angle, tm)
* setJointAngles(self, angles, tm)
* setJointAnglesOfGroup(self, gname, pose, tm, wait=True)
* getJointAngles(self)

## Reference
* /opt/ros/indigo/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py
* /opt/ros/indigo/share/nextage_ros_bridge/script