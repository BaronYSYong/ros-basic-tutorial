# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, print_function
import math
import numpy


# http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/
def axisangle2euler(x, y, z, angle):
    # expects (x, y, z) to be normalised

    sa = math.sin(angle)
    ca = math.cos(angle)
    aa = (1-ca)

    if (x*y*aa + z*sa) > 0.998: # northpole
        heading = 2*math.atan2(x*math.sin(angle/2.0), math.cos(angle/2.0))
        bank = 0
        attitude = math.pi/2

    elif (x*y*aa + z+sa) < -0.998: # southpole
        heading = -2*math.atan2(x*math.sin(angle/2.0), math.cos(angle/2.0))
        bank = 0
        attitude = -math.pi/2

    else:
        heading  = math.atan2(y*sa - x*z*aa, 1 - (y*y + z*z)*aa)
        attitude = math.asin(x*y*aa + z*sa)
        bank     = math.atan2(x*sa - y*z*aa, 1 - (x*x + z*z)*aa)

    # roll pitch yaw
    return bank, attitude, heading


# http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToAngle/
def euler2axisangle(bank, attitude, heading):
    c1 = math.cos(heading/2.0)
    c2 = math.cos(attitude/2.0)
    c3 = math.cos(bank/2.0)
    s1 = math.sin(heading/2.0)
    s2 = math.sin(attitude/2.0)
    s3 = math.sin(bank/2.0)

    angle = 2*math.acos(c1*c2*c3 - s1*s2*s3)
    x = s1*s2*c3 + c1*c2*s3
    y = s1*c2*c3 + c1*s2*s3
    z = c1*s2*c3 - s1*c2*s3
    length = math.sqrt(x*x + y*y + z*z)
    x /= length
    y /= length
    z /= length

    return x, y, z, angle


# http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
def axisangle2matrix(x, y, z, angle):
    # (x, y, z) must be normalized
    ca = math.cos(angle)
    sa = math.sin(angle)
    cc = (1 - ca)

    m00 = ca + x*x*cc
    m11 = ca + y*y*cc
    m22 = ca + z*z*cc

    tmp11 = x*y*cc
    tmp12 = z*sa
    m01 = tmp11 - tmp12
    m10 = tmp11 + tmp12

    tmp21 = x*z*cc
    tmp22 = y*sa
    m02 = tmp21 + tmp22
    m20 = tmp21 - tmp22

    tmp31 = y*z*cc
    tmp32 = x*sa
    m12 = tmp31 - tmp32
    m21 = tmp31 + tmp32

    return numpy.matrix([[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]])


# http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
def matrix2axisangle(m):
    epsilon  = 0.01
    epsilon2 = 0.1
    # should check that m is actually a rotation matrix
    if math.fabs(m[0,1] - m[1,0]) < epsilon and \
       math.fabs(m[0,2] - m[2,0]) < epsilon and \
       math.fabs(m[1,2] - m[2,1]) < epsilon:

        if math.fabs(m[0,1] - m[1,0]) < epsilon2 and \
           math.fabs(m[0,2] + m[2,0]) < epsilon2 and \
           math.fabs(m[1,2] + m[2,1]) < epsilon2 and \
           math.fabs(m[0,0] + m[1,1] + m[2,2] - 3) < epsilon2:
            # identity matrix
            return 0, 1, 0, 0

        else:

            angle = math.pi

            xx = (m[0,0] + 1)/2.0
            yy = (m[1,1] + 1)/2.0
            zz = (m[2,2] + 1)/2.0
            xy = (m[0,1] + m[1,0])/4.0
            xz = (m[0,2] + m[2,1])/4.0
            yz = (m[1,2] + m[2,1])/4.0

            if xx > yy and xx > zz: # m[0,0] is largest diagonal term
                if xx < epsilon:
                    x = .0
                    y = math.sqrt(2)/2.0
                    z = math.sqrt(2)/2.0

                else:
                    x = math.sqrt(xx)
                    y = xy/x
                    z = xz/x

            elif yy > zz: # m[1,1] is largest diagonal term
                if yy < epsilon:
                    x = math.sqrt(2)/2.0
                    y = .0
                    z = math.sqrt(2)/2.0

                else:
                    y = math.sqrt(yy)
                    x = xy/y
                    z = yz/y

            else: # m[2,2] is largest diagonal term
                if zz < epsilon:
                    x = math.sqrt(2)/2.0
                    y = math.sqrt(2)/2.0
                    z = .0

                else:
                    z = math.sqrt(zz)
                    x = xz/z
                    y = yz/z
            # 180deg rotation

    else:
        # no singularities
        s = math.sqrt((m[2,1] - m[1,2])*(m[2,1] - m[1,2]) +
                      (m[0,2] - m[2,0])*(m[0,2] - m[2,0]) +
                      (m[1,0] - m[0,1])*(m[1,0] - m[0,1]))

        angle = math.acos((m[0,0] + m[1,1] + m[2,2] - 1) /2.0)
        x = (m[2,1] - m[1,2])/s
        y = (m[0,2] - m[2,0])/s
        z = (m[1,0] - m[0,1])/s

    return x, y, z, angle

#
def rotationX(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return numpy.matrix([[1.0, 0.0, 0.0],
                         [0.0,   c,  -s],
                         [0.0,   s,   c]])

#
def rotationY(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return numpy.matrix([[  c, 0.0,   s],
                         [0.0, 1.0, 0.0],
                         [ -s, 0.0,   c]])

#
def rotationZ(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return numpy.matrix([[  c,  -s, 0.0],
                         [  s,   c, 0.0],
                         [0.0, 0.0, 1.0]])
