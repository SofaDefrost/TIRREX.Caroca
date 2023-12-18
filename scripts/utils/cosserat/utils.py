# Author Yinoussa Adagolodjo

import numpy as np
from math import log


def rotationMatrixX(theta):
    rotation = np.array([[1, 0, 0],
                         [0, np.cos(theta), -np.sin(theta)],
                         [0, np.sin(theta), np.cos(theta)]])
    return rotation


def rotationMatrixY(theta):
    rotation = np.array([[np.cos(theta), 0, np.sin(theta)],
                         [0, 1, 0],
                         [-np.sin(theta), 0, np.cos(theta)]])
    return rotation


def rotationMatrixZ(theta):
    rotation = np.array([[np.cos(theta), -np.sin(theta), 0],
                         [np.sin(theta), np.cos(theta), 0],
                         [0, 0, 1]])
    return rotation


def computeRotationMatrix(angles):
    rotation = np.dot(rotationMatrixZ(angles[2]), np.dot(rotationMatrixY(angles[1]), rotationMatrixX(angles[0])))
    return rotation


def computeTheta(x, gX):
    traceGx = np.trace(gX)
    if x <= np.finfo(float).eps:
        theta = 0.0
    else:
        theta = 1.0 / x * np.arccos((traceGx / 2.0) - 1)
    return theta


def piecewiseLogmap(curvAbs, gX):
    """

    Args:
        curvAbs:
        gX:

    Returns:

    """

    theta = computeTheta(curvAbs, gX)

    if theta == 0.0:
        xi_hat = 1.0 / curvAbs * (gX - np.identity(4))
    else:
        t0 = curvAbs * theta
        t1 = np.sin(t0)
        t2 = np.cos(t0)
        t3 = 2 * t1 * t2
        t4 = 1 - 2 * t1 ** 2
        t5 = t0 * t4

        gp2 = np.dot(gX, gX)
        gp3 = np.dot(gp2, gX)

        xi_hat = 1.0 / curvAbs * (0.125 *
                                  (1.0 / np.sin(t0 / 2.0) ** 3) *
                                  np.cos(t0 / 2.0) *
                                  ((t5 - t1) * np.identity(4) - (t0 * t2 + 2 * t5 - t1 - t3) * gX + (
                                              2 * t0 * t2 + t5 - t1 - t3) * gp2 - (t0 * t2 - t1) * gp3)
                                  )

    xci = np.array([xi_hat[2, 1], xi_hat[0, 2], xi_hat[1, 0], xi_hat[0, 3], xi_hat[1, 3], xi_hat[2, 3]])

    return xci


def getStrainFromAngles(angles, curvAbs):
    """

    Args:
        angles:
        curvAbs: abscissa curve

    Returns:

    """

    gX = np.zeros((4, 4), dtype=float)
    gX[0][3] = curvAbs  # to deploy the beam node and the rest part of transform is equal to null
    gX[3][3] = 1  # The homogeneous matrix

    gX[0:3, 0:3] = rotationMatrixX(angles[0])
    xcix = piecewiseLogmap(curvAbs, gX)

    gX[0:3, 0:3] = rotationMatrixY(angles[1])
    xciy = piecewiseLogmap(curvAbs, gX)

    gX[0:3, 0:3] = rotationMatrixZ(angles[2])
    xciz = piecewiseLogmap(curvAbs, gX)

    # gX[0:3, 0:3] = computeRotationMatrix(angles)
    # xci = piecewiseLogmap(curvAbs, gX)

    return [xcix[0], xciy[1], xciz[2]]
    # return [xci[0], xci[1], xci[2]]
