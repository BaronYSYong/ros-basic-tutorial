from nextage import NEXTAGE

class LAUNDROID(NEXTAGE, object):
    def MoveHandAbsolute(self, gname, pos, tm=1, wait=True):
        if gname == 'larm':
            eename = 'LARM_JOINT5'
        elif gname == 'rarm':
            eename = 'RARM_JOINT5'
        else:
            print "wrong gname"
            raise
        rpy = [round(elem, 2) for elem in self.getCurrentRPY(eename)]
        self.setTargetPose(gname, pos, rpy, tm=1)

if __name__ == '__main__':
    robot = LAUNDROID()
    robot.init()
