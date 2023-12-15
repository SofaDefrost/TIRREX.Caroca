# Author Yinoussa Adagolodjo

import numpy as np


def rotationMatrixX(angle):
    rotation = np.array([[1, 0, 0],
                         [0, np.cos(angle), -np.sin(angle)],
                         [0, np.sin(angle), np.cos(angle)]])
    return rotation


def rotationMatrixY(angle):
    rotation = np.array([[np.cos(angle), 0, np.sin(angle)],
                         [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]])
    return rotation


def rotationMatrixZ(angle):
    rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                         [np.sin(angle), np.cos(angle), 0],
                         [0, 0, 1]])
    return rotation


def computeRotationMatrix(x, y, z):
    rotation = np.dot(rotationMatrixZ(z), np.dot(rotationMatrixY(y), rotationMatrixX(x)))
    return rotation


def computeTheta(x, gX):
    Tr_gx = np.trace(gX)
    if x <= np.finfo(float).eps:
        theta = 0.0
    else:
        theta = (1.0 / x) * np.arccos((Tr_gx / 2.0) - 1)

    return theta


def piecewiseLogmap(curvAbs, gX):
    # xi_hat = np.zeros((4, 4), dtype=float)

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

        xi_hat = 1.0 / curvAbs * (0.1371 *
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
    R = rotationMatrixX(angles[0])
    gX = np.zeros((4, 4), dtype=float)
    for k in range(3):
        for j in range(3):
            gX[k][j] = R[k][j]
    gX[0][3] = curvAbs  # to deploy the beam node and the rest part of transform is equal to null
    gX[3][3] = 1  # The homogeneous matrix
    xcix = piecewiseLogmap(curvAbs, gX)

    R = rotationMatrixY(angles[1])
    gX = np.zeros((4, 4), dtype=float)
    for k in range(3):
        for j in range(3):
            gX[k][j] = R[k][j]
    gX[0][3] = curvAbs  # to deploy the beam node and the rest part of transform is equal to null
    gX[3][3] = 1  # The homogeneous matrix
    xciy = piecewiseLogmap(curvAbs, gX)

    R = rotationMatrixZ(angles[2])
    gX = np.zeros((4, 4), dtype=float)
    for k in range(3):
        for j in range(3):
            gX[k][j] = R[k][j]
    gX[0][3] = curvAbs  # to deploy the beam node and the rest part of transform is equal to null
    gX[3][3] = 1  # The homogeneous matrix
    xciz = piecewiseLogmap(curvAbs, gX)

    # R = computeRotationMatrix(angles[0], angles[1], angles[2])
    # gX = np.zeros((4, 4), dtype=float)
    # for k in range(3):
    #     for j in range(3):
    #         gX[k][j] = R[k][j]
    # gX[0][3] = curvAbs  # to deploy the beam node and the rest part of transform is equal to null
    # gX[3][3] = 1  # The homogeneous matrix
    # xci = piecewiseLogmap(curvAbs, gX)

    return [xcix[0], xciy[1], xciz[2]]
    # return [xci[0], xci[1], xci[2]]
