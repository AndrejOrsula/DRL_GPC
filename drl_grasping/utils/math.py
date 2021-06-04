from scipy.spatial.transform import Rotation
from typing import Tuple
import math
import numpy


def quat_mul(quat_0: Tuple[float, float, float, float],
             quat_1: Tuple[float, float, float, float],
             xyzw: bool = True) -> Tuple[float, float, float, float]:
    """
    Multiply two quaternions
    """
    if xyzw:
        x0, y0, z0, w0 = quat_0
        x1, y1, z1, w1 = quat_1
        return (x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0)
    else:
        w0, x0, y0, z0 = quat_0
        w1, x1, y1, z1 = quat_1
        return (-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0)


def rot_z(vector, theta) -> numpy.ndarray:
    """Rotates 3D vector around z-axis"""
    return Rotation.from_rotvec((0, 0, theta)).apply(list(vector))


def rpy2rotvec(rpy: Tuple[float, float, float]) -> Tuple[float, float, float]:

    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    alpha = yaw
    beta = pitch
    gamma = roll

    ca = math.cos(alpha)
    cb = math.cos(beta)
    cg = math.cos(gamma)
    sa = math.sin(alpha)
    sb = math.sin(beta)
    sg = math.sin(gamma)

    r11 = ca*cb
    r12 = ca*sb*sg-sa*cg
    r13 = ca*sb*cg+sa*sg
    r21 = sa*cb
    r22 = sa*sb*sg+ca*cg
    r23 = sa*sb*cg-ca*sg
    r31 = -sb
    r32 = cb*sg
    r33 = cb*cg

    theta = math.acos((r11+r22+r33-1)/2)
    sth = math.sin(theta)
    kx = (r32-r23)/(2*sth)
    ky = (r13-r31)/(2*sth)
    kz = (r21-r12)/(2*sth)

    return ((theta*kx), (theta*ky), (theta*kz))


def rotvec2rpy(rotvec: Tuple[float, float, float]) -> Tuple[float, float, float]:
    theta = math.sqrt(rotvec[0]*rotvec[0]+rotvec[1]
                      * rotvec[1]+rotvec[2]*rotvec[2])

    k1 = rotvec[0]/theta
    k2 = rotvec[1]/theta
    k3 = rotvec[2]/theta
    r11 = k1*k1*(1-math.cos(theta))+math.cos(theta)
    # r12 = k1*k2*(1-math.cos(theta))-k3*math.sin(theta)
    # r13 = k1*k3*(1-math.cos(theta))+k2*math.sin(theta)
    r21 = k1*k2*(1-math.cos(theta))+k3*math.sin(theta)
    r22 = k2*k2*(1-math.cos(theta))+math.cos(theta)
    r23 = k2*k3*(1-math.cos(theta))-k1*math.sin(theta)
    r31 = k1*k3*(1-math.cos(theta))-k2*math.sin(theta)
    r32 = k2*k3*(1-math.cos(theta))+k1*math.sin(theta)
    r33 = k3*k3*(1-math.cos(theta))+math.cos(theta)

    if r31 < 1:
        if r31 > -1:
            roll = math.atan2(r21, r11)
            pitch = math.asin(-r31)
            yaw = math.atan2(r32, r33)
        else:
            roll = -math.atan2(-r23, r22)
            pitch = math.pi/2
            yaw = 0
    else:
        roll = math.atan2(-r23, r22)
        pitch = -math.pi/2
        yaw = 0

    return (roll, pitch, yaw)
