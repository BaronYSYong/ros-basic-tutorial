from laundroid import LAUNDROID
from vision import VISION

robot = LAUNDROID()
robot.init()

vision = VISION()

for i in range(199):
    robot.goInitial(tm=1)
    pos = vision.hold_point()
    robot.MoveHandAbsolute('rarm', pos)
