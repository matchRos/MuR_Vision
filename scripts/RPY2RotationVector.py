#!/usr/bin/env python
import math
import numpy as np

def rpy2rv(roll,pitch,yaw):

    yawMatrix = np.matrix([
    [math.cos(yaw), -math.sin(yaw), 0],
    [math.sin(yaw), math.cos(yaw), 0],
    [0, 0, 1]
    ])

    pitchMatrix = np.matrix([
    [math.cos(pitch), 0, math.sin(pitch)],
    [0, 1, 0],
    [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    rollMatrix = np.matrix([
    [1, 0, 0],
    [0, math.cos(roll), -math.sin(roll)],
    [0, math.sin(roll), math.cos(roll)]
    ])

    R = yawMatrix * pitchMatrix * rollMatrix

    # theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
    # Due to the value range of arccos is [0, pi], but theta can be negative.
    # Therefore, using acos can not determine theta correctly. ---> use atan2
    cos_theta = ((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2
    sin_theta = math.sqrt((R[2,1]- R[1,2])**2 + (R[0, 2] - R[2, 0])**2 + (R[1, 0] - R[0, 1])**2)/2
    theta = math.atan2(sin_theta, cos_theta)

    if math.sin(theta) < 0.0001:  # sin(theta) = 0
        if math.fabs(theta) < 0.0001: # theta = 0
            rx = 0
            ry = 0
            rz = 0
        elif math.fabs(math.fabs(theta) - math.pi) < 0.0001: # theta = pi or -pi
            ux = math.sqrt((R[0,0] + 1) /2)
            uy = math.sqrt((R[1,1] + 1) /2)
            uz = math.sqrt((R[2,2] + 1) /2)
            if R[0,1] < 0:
                uy = -uy
            if R[0,2] < 0:
                uz = -uz
            rx = ux * theta
            ry = uy * theta
            rz = uz * theta
    else:
        multi = 1 / (2 * math.sin(theta))
        rx = multi * (R[2, 1] - R[1, 2]) * theta
        ry = multi * (R[0, 2] - R[2, 0]) * theta
        rz = multi * (R[1, 0] - R[0, 1]) * theta
    return rx,ry,rz

if __name__ == '__main__':
    roll =  0
    pitch = -2.212
    yaw =   1.15

    (rx, ry, rz) = rpy2rv(roll, pitch,yaw)
