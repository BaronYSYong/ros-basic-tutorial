from hironx_ros_bridge.ros_client import ROS_Client
from nextage_ros_bridge.nextage_client import NextageClient

from hrpsys import rtm
import argparse

import math
import numpy
import transformations
from hrpsys.hrpsys_config import euler_from_matrix

class NEXTAGE(NextageClient, object):
    def connect(cls):
        '''!@brief
        Modified from 
        /opt/ros/indigo/share/nextage_ros_bridge/script/nextage.py
        '''        
        parser = argparse.ArgumentParser(description='hiro command line interpreters')
        parser.add_argument('--host', help='corba name server hostname')
        parser.add_argument('--port', help='corba name server port number')
        parser.add_argument('--modelfile', help='robot model file nmae')
        parser.add_argument('--robot', help='robot modlule name (RobotHardware0 for real robot, Robot()')
        parser.add_argument('--dio_ver', help="Version of digital I/O. Only users "
                            "whose robot was shipped before Aug 2014 need to "
                            "define this, and the value should be '0.4.2'.")
        args, unknown = parser.parse_known_args()
        
        print args
        print unknown

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
            
    def circle_eef(self, radius=0.01, eef='larm', step_degree=5, ccw=True, duration=0.1):
        '''
        Moves the designated eef point-by-point so that the trajectory as a whole draws a circle.

        Currently this only works on the Y-Z plane of *ARM_JOINT5 joint.
        And it's the most intuitive when eef maintains a "goInitial" pose where circle gets drawn on robot's X-Y plane
        (see the wiki for the robot's coordinate if you're confused http://wiki.ros.org/rtmros_nextage/Tutorials/Programming#HiroNXO_3D_model_coordination).

        Points on the circular trajectory is based on a standard equation https://en.wikipedia.org/wiki/Circle#Equations

        @param radius: (Unit: meter) Radius of the circle to be drawn.
        @param step_degree: Angle in degree each iteration increments.
        @param ccw: counter clock-wise.
        @param duration: Time for each iteration to be completed.
        '''
        goal_deg = GOAL_DEGREE = 360
        start_deg = 0
        if eef == 'larm': 
            joint_eef = 'LARM_JOINT5'
        elif eef == 'rarm':
            joint_eef = 'RARM_JOINT5'
        eef_pos = self.getCurrentPosition(joint_eef)
        eef_rpy = self.getCurrentRPY(joint_eef)
        print('eef_pos={}'.format(eef_pos))
        X0 = eef_pos[0]
        Y0 = eef_pos[1]
        ORIGIN_x = X0
        ORIGIN_y = Y0 - radius
        print('ORIGIN_x={} ORIGIN_y={}'.format(ORIGIN_x, ORIGIN_y))
        i = 0
        for theta in range(start_deg, goal_deg, step_degree):
            if not ccw:
                theta = -theta
            x = ORIGIN_x + radius*math.sin(math.radians(theta))  # x-axis in robot's eef space is y in x-y graph
            y = ORIGIN_y + radius*math.cos(math.radians(theta))
            eef_pos[0] = x
            eef_pos[1] = y
            print('#{}th theta={} x={} y={} X0={} Y0={}'.format(i, theta, x, y, X0, Y0))
            self.setTargetPose(eef, eef_pos, eef_rpy, duration)
            self.waitInterpolation()
            i += 1

    def setTargetPoseAxisAngleRelativeLocal(self, gname, eename,
                                            dx=0, dy=0, dz=0, axis=[0, 0, 1],
                                            dangle=0, tm=10, wait=True):
        '''!@brief
        Set target pose axis angle relative according to local coordinate(end-effector).

        For d*, distance arguments are in meter; while rotations, dangle is in radians.

        Example usage: The following moves LARM_JOINT5 joint 1 radian CCW according to z-axis based on
        end-effector within 3 sec.

        \verbatim
            robot.setTargetPoseAxisAngleRelativeLocal('larm', 'LARM_JOINT5',
                                                  dx=0, dy=0, dz=0, axis=[0, 0, 1],
                                                  dangle=1, tm=3, wait=True)
        \endverbatim
        @param gname str: Name of the joint group.
        @param eename str: Name of end-effector.
        @param dx float: In meter.
        @param dy float: In meter.
        @param dz float: In meter.
        @param axis list: 1 or 0, e.g. axis=[0,0,1] means turning according z-axis
        @param dangle float: In radian.
        @param tm float: Second to complete.
        @param wait bol: If true, SequencePlayer.waitInterpolationOfGroup gets run.
        @return bool: False if unreachable.
        '''
        self.waitInterpolationOfGroup(gname)
        tds = self.getCurrentPose(eename)
        drot = transformations.axisangle2matrix(axis[0], axis[1], axis[2], dangle)
        rotRef = numpy.matrix([tds[0:3], tds[4:7], tds[8:11]])
        rotRef = rotRef * drot
        rpyRef = euler_from_matrix(rotRef)
        tds_mat = numpy.matrix([tds[0:4], tds[4:8], tds[8:12], tds[12:16]])
        daxis = numpy.matrix([[dx], [dy], [dz], [1]])
        posRef = tds_mat*daxis
        posRef = posRef.tolist()
        posRef = [posRef[0][0], posRef[1][0], posRef[2][0]]
        ret = self.setTargetPose(gname, posRef, rpyRef, tm, frame_name=None)
        if ret and wait:
            self.waitInterpolationOfGroup(gname)
        return ret


    def setTargetPoseAxisAngleRelativeLocalOffset(self, gname, eename, offset=[0, 0, 0],
                                                  dx=0, dy=0, dz=0, axis=[0, 0, 1],
                                                  dangle=0, tm=10, wait=True):
        '''!@brief
        Set target pose axis angle relative according to local coordinate which based on offset from end-effector.

        For d*, distance arguments are in meter; while rotations, dangle is in radians.

        Example usage: The following moves LARM_JOINT5 joint 1 radian CCW according to z-axis based on
        (-0.2, 0, 0) offset from end-effector within 3 sec.

        \verbatim
            robot.setTargetPoseAxisAngleRelativeLocalOffset('larm', 'LARM_JOINT5', offset=[-0.2, 0, 0],
                                                  dx=0, dy=0, dz=0, axis=[0, 0, 1],
                                                  dangle=1, tm=3, wait=True)
        \endverbatim
        @param gname str: Name of the joint group.
        @param eename str: Name of end-effector.
        @param offset list: In meter, e.g. offset=[-0.2, 0,0] means the reference point is offset x=-0.2 m from end-effector
        @param dx float: In meter.
        @param dy float: In meter.
        @param dz float: In meter.
        @param axis list: 1 or 0, e.g. axis=[0,0,1] means turning according z-axis
        @param dangle float: In radian.
        @param tm float: Second to complete.
        @param wait bol: If true, SequencePlayer.waitInterpolationOfGroup gets run.
        @return bool: False if unreachable.
        '''
        self.waitInterpolationOfGroup(gname)
        tds = self.getCurrentPose(eename)
        rot_now_global = numpy.matrix([tds[0:3], tds[4:7], tds[8:11]])
        pos_now_global = numpy.matrix([[tds[3]], [tds[7]], [tds[11]]])
        tip_offset_local = numpy.matrix([[offset[0]], [offset[1]], [offset[2]]])
        rot_new_local = transformations.axisangle2matrix(axis[0], axis[1], axis[2], dangle)
        tooltip_offset_global = rot_now_global*tip_offset_local
        tooltip_pos_global = pos_now_global + tooltip_offset_global
        rot_new_global = rot_now_global*rot_new_local
        pos_new_global = tooltip_pos_global + (rot_new_global*(-tip_offset_local))
        rpyRef = euler_from_matrix(rot_new_global)
        rot_list = rot_new_global.tolist()
        pos_list = pos_new_global.tolist()
        trans_list = rot_list
        for i in range(3):
            trans_list[i].append(pos_list[i][0])
        trans_list.append([0,0,0,1])
        trans_mat = numpy.matrix(trans_list)
        daxis = numpy.matrix([[dx], [dy], [dz], [1]])
        posRef = trans_mat*daxis
        posRef = posRef.tolist()
        posRef = [posRef[0][0], posRef[1][0], posRef[2][0]]
        ret = self.setTargetPose(gname, posRef, rpyRef, tm, frame_name=None)
        if ret and wait:
            self.waitInterpolationOfGroup(gname)
        return ret

if __name__ == '__main__':
    robot = NEXTAGE()
    robot.init()
