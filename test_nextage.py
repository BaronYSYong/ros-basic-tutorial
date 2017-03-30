from nextage import NEXTAGE

robot = NEXTAGE.connect()
robot.goOffPose()
robot.goInitial()
robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dp=-1, tm=1, wait=False)
robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dp=-1, tm=1, wait=True)
for i in range(100):
    robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=0.1, tm=1, wait=False)
    robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=0.1, tm=1, wait=True)
    robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=-0.1, tm=1, wait=False)
    robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=-0.1, tm=1, wait=True)
