"""
http://wiki.ros.org/rtmros_nextage/Tutorials
http://wiki.ros.org/rtmros_nextage/Tutorials/Programming_Hiro_NEXTAGE_OPEN_RTM
"""

from hironx_ros_bridge.ros_client import ROS_Client
from nextage_ros_bridge import nextage_client

from hrpsys import rtm
import argparse

class NEXTAGE(object):
    def __init__(self):
        parser = argparse.ArgumentParser(description='hiro command line interpreters')
        parser.add_argument('--host', help='corba name server hostname')
        parser.add_argument('--port', help='corba name server port number')
        parser.add_argument('--modelfile', help='robot model file nmae')
        parser.add_argument('--robot', help='robot modlule name (RobotHardware0 for real robot, Robot()')
        parser.add_argument('--dio_ver', help="Version of digital I/O. Only users "
                            "whose robot was shipped before Aug 2014 need to "
                            "define this, and the value should be '0.4.2'.")
        args, unknown = parser.parse_known_args()

        if args.host:
            rtm.nshost = args.host
        if args.port:
            rtm.nsport = args.port
        if not args.robot:
            args.robot = "RobotHardware0" if args.host else "HiroNX(Robot)0"
        if not args.modelfile:
            args.modelfile = "/opt/jsk/etc/NEXTAGE/model/main.wrl" if args.host else "" 

        # support old style format
        if len(unknown) >= 2:
            args.robot = unknown[0]
            args.modelfile = unknown[1]
        self.robot = nxc = nextage_client.NextageClient()
        self.robot.init(robotname=args.robot, url=args.modelfile)

        if args.dio_ver:
            self.robot.set_hand_version(args.dio_ver)        
        
if __name__ == '__main__':
    robot = NEXTAGE()


