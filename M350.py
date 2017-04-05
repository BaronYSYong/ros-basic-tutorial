from laundroid import LAUNDROID

robot = LAUNDROID()
robot.init()

for i in range(100):
    robot.goInitial(tm=1)
    robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dx=0.05, dy=-0.1, dz=-0.1, tm=1, wait=False)
    robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dx=0.05, dy=0.1, dz=-0.1, tm=1, wait=True)
    robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=0.1, tm=1, wait=False)
    robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=0.1, tm=1, wait=True)
