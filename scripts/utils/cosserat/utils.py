# Author Yinoussa Adagolodjo

import numpy as np


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
        xiHat = 1.0 / curvAbs * (gX - np.identity(4))
    else:
        t0 = curvAbs * theta
        t1 = np.sin(t0)
        t2 = np.cos(t0)
        t3 = 2 * t1 * t2
        t4 = 1 - 2 * t1 ** 2
        t5 = t0 * t4

        gX2 = np.dot(gX, gX)
        gX3 = np.dot(gX2, gX)

        c0 = (t5 - t1)
        c1 = (t0 * t2 + 2 * t5 - t1 - t3)
        c2 = (2 * t0 * t2 + t5 - t1 - t3)
        c3 = (t0 * t2 - t1)

        xiHat = 1.0 / curvAbs * (0.125 *
                                 (1.0 / np.sin(t0 / 2.0) ** 3) *
                                 np.cos(t0 / 2.0) *
                                 (c0 * np.identity(4) - c1 * gX + c2 * gX2 - c3 * gX3)
                                 )

    xi = np.array([xiHat[2, 1], xiHat[0, 2], xiHat[1, 0], xiHat[0, 3], xiHat[1, 3], xiHat[2, 3]])

    return xi


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
    xix = piecewiseLogmap(curvAbs, gX)

    gX[0:3, 0:3] = rotationMatrixY(angles[1])
    xiy = piecewiseLogmap(curvAbs, gX)

    gX[0:3, 0:3] = rotationMatrixZ(angles[2])
    xiz = piecewiseLogmap(curvAbs, gX)

    # gX[0:3, 0:3] = computeRotationMatrix(angles)
    # xi = piecewiseLogmap(curvAbs, gX)

    return [xix[0], xiy[1], xiz[2]]
    # return [xi[0:3]]
