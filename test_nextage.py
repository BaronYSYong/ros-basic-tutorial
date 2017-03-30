from nextage import NEXTAGE
from vision import VISION

robot = NEXTAGE()
robot.init()

vision = VISION()

robot.goInitial()
pos = vision.hold_point()
rpy = [round(elem, 2) for elem in robot.getCurrentRPY('RARM_JOINT5')]
robot.setTargetPose('RARM_JOINT5', pos, rpy, tm=1)


#~ robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dp=-1, tm=1, wait=False)
#~ robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dp=-1, tm=1, wait=True)
#~ for i in range(100):
    #~ robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=0.1, tm=1, wait=False)
    #~ robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=0.1, tm=1, wait=True)
    #~ robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=-0.1, tm=1, wait=False)
    #~ robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=-0.1, tm=1, wait=True)

#~ robot.setTargetPoseAxisAngleRelativeLocalOffset('larm', 'LARM_JOINT5', offset=[-0.2, 0, 0],
                                                  #~ dx=0, dy=0, dz=0, axis=[0, 0, 1],
                                                  #~ dangle=1, tm=3, wait=True)
