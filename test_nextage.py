from nextage import NEXTAGE

robot = NEXTAGE()
robot.init()
robot.goOffPose()
robot.goInitial()
robot.setTargetPoseAxisAngleRelativeLocalOffset('larm', 'LARM_JOINT5', offset=[-0.2, 0, 0],
                                                  dx=0, dy=0, dz=0, axis=[0, 0, 1],
                                                  dangle=1, tm=3, wait=True)
#~ robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dp=-1, tm=1, wait=False)
#~ robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dp=-1, tm=1, wait=True)
#~ robot.circle_eef(radius=0.1)
#~ for i in range(100):
    #~ robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=0.1, tm=1, wait=False)
    #~ robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=0.1, tm=1, wait=True)
    #~ robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=-0.1, tm=1, wait=False)
    #~ robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=-0.1, tm=1, wait=True)
