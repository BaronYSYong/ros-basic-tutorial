from hironx_ros_bridge.ros_client import ROS_Client
from nextage_ros_bridge import nextage_client
from hrpsys import rtm
import argparse


if __name__ == '__main__':
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

    if len(unknown) >= 2:
        args.robot = unknown[0]
        args.modelfile = unknown[1]
    robot = nxc = nextage_client.NextageClient()
    robot.init(robotname=args.robot, url=args.modelfile)

    if args.dio_ver:
        robot.set_hand_version(args.dio_ver)

    """
    Reference:
    /opt/ros/indigo/lib/python2.7/dist-packages/hrpsys/hrpsys_config.py
    /opt/ros/indigo/share/nextage_ros_bridge/script
    
    robot.setTargetPose('rarm', [0.32, -0.18, 0.06], [-3, -1.5, 3.0], tm=3)
    robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=0.01, tm=3)
    """
    #~ robot.goOffPose()
    robot.goInitial()
    robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dp=-1, tm=1, wait=False)
    robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dp=-1, tm=1, wait=True)
    for i in range(100):
        robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=0.1, tm=1, wait=False)
        robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=0.1, tm=1, wait=True)
        robot.setTargetPoseRelative('larm', 'LARM_JOINT5', dz=-0.1, tm=1, wait=False)
        robot.setTargetPoseRelative('rarm', 'RARM_JOINT5', dz=-0.1, tm=1, wait=True)
