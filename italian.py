from laundroid import LAUNDROID
import math

robot = LAUNDROID()
robot.init()

robot.goInitial(tm=1)

#~ angles = [[0], [0, 0],
            #~ [-0.6, 0, -100, -180.0+15.2, 9.4, 3.2],
            #~ [0.6, 0, -100, 180.0-15.2, 9.4, -3.2],
            #~ [0, 0, 0, 0],
            #~ [0, 0, 0, 0]]

#~ for i in range(len(robot.Groups)):
    #~ robot.setJointAnglesOfGroup(robot.Groups[i][0], angles[i], tm=5, wait=False)

#~ for i in range(100):
    #~ robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dp=-0.3, tm=1, wait=False)
    #~ robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dp=-0.3, tm=1, wait=True)
    #~ robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dp=0.3, tm=1, wait=False)
    #~ robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dp=0.3, tm=1, wait=True)


LARM = [-0.6, 0, -100, -180.0+15.2, 9.4, 3.2]
RARM = [0.6, 0, -100, 180.0-15.2, 9.4, -3.2]
LARM_2 = [-0.6, 0, -100, -180.0+15.2, 30.0, 3.2]
RARM_2 = [0.6, 0, -100, 180.0-15.2, 30.0, -3.2]

robot.setJointAnglesOfGroup(robot.Groups[2][0], LARM, tm=5, wait=False)
robot.setJointAnglesOfGroup(robot.Groups[3][0], RARM, tm=5, wait=True)

for i in range(100):
    robot.setJointAnglesOfGroup(robot.Groups[2][0], LARM_2, tm=1, wait=False)
    robot.setJointAnglesOfGroup(robot.Groups[3][0], RARM_2, tm=1, wait=True)
    robot.setJointAnglesOfGroup(robot.Groups[2][0], LARM, tm=1, wait=False)
    robot.setJointAnglesOfGroup(robot.Groups[3][0], RARM, tm=1, wait=True)
