# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function

from hrpsys import rtm
from nextage_ros_bridge.nextage_client import NextageClient

from utility import XMUtility


class NEXTAGE(NextageClient, XMUtility):
    def __init__(self, name):
        NextageClient.__init__(self)
        XMUtility.__init__(self, name)

    @classmethod
    def connect(cls):
        import argparse
        import roslib; roslib.load_manifest('nextage_ros_bridge')

        parser = argparse.ArgumentParser(description='nextage command line interpreter')
        parser.add_argument('--host', type=str, help='corba name server hostname')
        parser.add_argument('--port', type=int, help='corba name server port number')
        parser.add_argument('--modelfile', type=str, default='/opt/jsk/etc/HIRONX/model/main.wrl', help='robot model file nmae')
        parser.add_argument('--name', type=str, default='nextage', help='robot name')
        args, unknown = parser.parse_known_args()

        if args.host:
            rtm.nshost = args.host
        if args.port:
            rtm.nsport = args.port

        robot = NEXTAGE(args.name)
        if args.host:
            robot.init(robotname=args.name, url=args.modelfile)
        else:
            robot.init()

        # Change base CS origin to chest position to be the same as
        # HIRONX and XMRobot.
        robot.setJointAnglesOfGroupRelative('head', [0, 0], tm=0.1)
        robot.seq_svc.setBasePos([0, 0, 0], 0.1)

        return robot


if __name__ == '__main__':
    nxc = robot = NEXTAGE.connect()
