# Author Yinoussa Adagolodjo

import numpy as np
from scipy.linalg import logm


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


def piecewiseLogmap(curvAbs, gX):
    """

    Args:
        curvAbs:
        gX:

    Returns:

    """

    curvAbsInv = 1 / curvAbs
    xiHat = curvAbsInv * logm(gX)

    xi = np.array([xiHat[2][1], xiHat[0][2], xiHat[1][0], xiHat[0][3], xiHat[1][3], xiHat[2][3]])

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
