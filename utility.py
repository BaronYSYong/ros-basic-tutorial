# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function
import math

import numpy
from hrpsys.hrpsys_config import euler_from_matrix

import transformations

class XMUtility(object):
    # heading to be used for heading coordinates
    heading = 0.0

    # to be AddarmsHandClient
    rhand = None
    lhand = None

    def __init__(self, name):
        self.ROBOT_TYPE = self.__class__.__name__
        self.name = name

    def getCurrentAxisAngle(self, joint):
        return transformations.euler2axisangle(*self.getCurrentRPY(joint))

    def getJointAnglesOfGroup(self, group):
        angles = self.getJointAngles()

        if group == 'torso':
            return angles[0]
        elif group == 'head':
            return angles[1:3]
        elif group == 'rarm':
            return angles[3:9]
        elif group == 'larm':
            return angles[9:15]
        else:
            return None

    def setJointAnglesOfGroupRelative(self, group, delta, tm, wait=True):
        joints = self.getJointAngles()

        if group == 'torso':
            joints = [joints[0]]
        elif group == 'head':
            joints = joints[1:3]
        elif group == 'rarm':
            joints = joints[3:9]
        else:
            joints = joints[9:15]

        for i, d in enumerate(delta):
            joints[i] += d
        return self.setJointAnglesOfGroup(group, joints, tm, wait)

    def setHeading(self, heading):
        self.heading = math.radians(heading)

    def setTargetPoseHeading(self, gname, pos, rpy, tm, frame_name=None):
        cs = math.cos(self.heading)
        sn = math.sin(self.heading)
        dx1 = pos[0] * cs - pos[1] * sn
        dy1 = pos[0] * sn + pos[1] * cs
        pos2 = (dx1, dy1, pos[2])
        rpy2 = (rpy[0], rpy[1], rpy[2] + self.heading)

        return self.setTargetPose(gname, pos2, rpy2, tm, frame_name)

    def setTargetPoseRelativeHeading(self, gname, eename,
                                     dx=0, dy=0, dz=0, dr=0, dp=0, dw=0,
                                     tm=10, wait=True, frame_name=None):
        cs = math.cos(self.heading)
        sn = math.sin(self.heading)
        dx1 = dx * cs - dy * sn
        dy1 = dx * sn + dy * cs

        return self.setTargetPoseRelative(gname, eename, dx1, dy1, dz,
                                          dr, dp, dw, tm, wait, frame_name)

    def setTargetPoseMatrix(self, gname, pos, rot, tm, frame_name=None):
        if not isinstance(rot, numpy.matrixlib.defmatrix.matrix):
            return None

        rpy = euler_from_matrix(rot)
        return self.setTargetPose(gname, pos, rpy, tm, frame_name)

    def setTargetPoseMatrixRelative(self, gname, eename,
                                    dx=0.0, dy=0.0, dz=0.0,
                                    drot=numpy.matrix([[1.0, 0.0, 0.0],
                                                       [0.0, 1.0, 0.0],
                                                       [0.0, 0.0, 1.0]]),
                                    tm=10, wait=True, frame_name=None):
        self.waitInterpolationOfGroup(gname)
        tds = self.getCurrentPose(eename)
        posRef = numpy.array([tds[3], tds[7], tds[11]])
        rotRef = numpy.matrix([tds[0:3], tds[4:7], tds[8:11]])
        daxis = numpy.array([dx, dy, dz])
        posRef += daxis
        rotRef = rotRef * drot
        rpyRef = euler_from_matrix(rotRef)
        ret = self.setTargetPose(gname, list(posRef), rpyRef, tm, frame_name)
        if ret and wait:
            self.waitInterpolationOfGroup(gname)
        return ret

    def setTargetPoseAxisAngle(self, gname, pos, axis, angle, tm=10, frame_name=None):
        rot = transformations.axisangle2matrix(axis[0], axis[1], axis[2], angle)
        return self.setTargetPoseMatrix(gname, pos, rot, tm, frame_name)

    def setTargetPoseAxisAngleRelative(self, gname, eename, dx=0, dy=0, dz=0,
                                       axis=[0, 0, 1], dangle=0, tm=10, wait=True):
        drot = transformations.axisangle2matrix(axis[0], axis[1], axis[2], dangle)
        return self.setTargetPoseMatrixRelative(gname, eename, dx, dy, dz,
                                                drot, tm, wait)

    def setTargetPoseAxisAngleHeading(self, gname, pos, axis, angle, tm):
        cs = math.cos(self.heading)
        sn = math.sin(self.heading)
        dx1 = pos[0] * cs - pos[1] * sn
        dy1 = pos[0] * sn + pos[1] * cs
        pos2 = (dx1, dy1, pos[2])
        rot = transformations.axisangle2matrix(axis[0], axis[1], axis[2], angle)
        rot2 = transformations.rotationZ(self.heading) * rot

        return self.setTargetPoseMatrix(gname, pos2, rot2, tm)

    def setTargetPoseAxisAngleRelativeHeading(self, gname, eename,
                                              dx=0, dy=0, dz=0, axis=[0, 0, 1],
                                              dangle=0, tm=10, wait=True):
        cs = math.cos(self.heading)
        sn = math.sin(self.heading)
        dx1 = dx * cs - dy * sn
        dy1 = dx * sn + dy * cs
        drot = transformations.axisangle2matrix(axis[0], axis[1], axis[2], dangle)

        return self.setTargetPoseMatrixRelative(gname, eename, dx1, dy1, dz,
                                                drot, tm, wait)

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
