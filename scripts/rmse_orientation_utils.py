'''
Functions for calculating orientation error.

Taken from https://github.com/dlaidig/broad/blob/main/example_code/broad_utils.py.
'''
import numpy as np

def quatmult(q1, q2):
    """
    Quaternion multiplication.

    If two Nx4 arrays are given, they are multiplied row-wise. Alternative one of the inputs can be a single
    quaternion which is then multiplied to all rows of the other input array.
    """

    q1 = np.asarray(q1, float)
    q2 = np.asarray(q2, float)

    # if both input quaternions are 1D arrays, we also want to return a 1D output
    is1D = max(len(q1.shape), len(q2.shape)) < 2

    # but to be able to use the same indexing in all cases, make sure everything is in 2D arrays
    if q1.shape == (4,):
        q1 = q1.reshape((1, 4))
    if q2.shape == (4,):
        q2 = q2.reshape((1, 4))

    # check the dimensions
    N = max(q1.shape[0], q2.shape[0])
    assert q1.shape == (N, 4) or q1.shape == (1, 4)
    assert q2.shape == (N, 4) or q2.shape == (1, 4)

    # actual quaternion multiplication
    q3 = np.zeros((N, 4), np.float64)
    q3[:, 0] = q1[:, 0] * q2[:, 0] - q1[:, 1] * q2[:, 1] - q1[:, 2] * q2[:, 2] - q1[:, 3] * q2[:, 3]
    q3[:, 1] = q1[:, 0] * q2[:, 1] + q1[:, 1] * q2[:, 0] + q1[:, 2] * q2[:, 3] - q1[:, 3] * q2[:, 2]
    q3[:, 2] = q1[:, 0] * q2[:, 2] - q1[:, 1] * q2[:, 3] + q1[:, 2] * q2[:, 0] + q1[:, 3] * q2[:, 1]
    q3[:, 3] = q1[:, 0] * q2[:, 3] + q1[:, 1] * q2[:, 2] - q1[:, 2] * q2[:, 1] + q1[:, 3] * q2[:, 0]

    if is1D:
        q3 = q3.reshape((4,))

    return q3

def invquat(q):
    """Calculates the inverse of unit quaternions."""

    q = np.asarray(q, np.float64)
    if len(q.shape) != 2:
        assert q.shape == (4,)
        qConj = q.copy()
        qConj[1:] *= -1
        return qConj
    else:
        assert q.shape[1] == 4
        qConj = q.copy()
        qConj[:, 1:] *= -1
        return qConj

def calculateErrorQuatEarth(imu_quat, opt_quat):
    """
    Calculates quaternion that represents the orientation estimation error in the global coordinate system.

    :param imu_quat: IMU orientation, shape (N, 4)
    :param opt_quat: OMC orientation, shape (N, 4)
    :return: error quaternion, shape (N, 4)
    """
    # normalize the input quaternions just in case
    imu_quat = imu_quat / np.linalg.norm(imu_quat, axis=1)[:, None]
    opt_quat = opt_quat / np.linalg.norm(opt_quat, axis=1)[:, None]
    # calculate the relative orientation expressed in the global coordinate system
    # imu_quat * (inv(opt_quat) * imu_quat) * inv(imu_quat) = imu_quat * inv(opt_quat)
    out = quatmult(imu_quat, invquat(opt_quat))
    # normalize the output quaternion
    out = out / np.linalg.norm(out, axis=1)[:, None]
    return out

def calculateTotalError(q_diff):
    """
    Calculates the total error, i.e. the total absolute rotation angle of the quaternion.

    :param q_diff: error quaternion, shape (N, 4)
    :return: error in rad, shape (N,)
    """
    return 2 * np.arccos(np.clip(np.abs(q_diff[:, 0]), 0, 1))


def calculateHeadingError(q_diff_earth):
    """
    Calculates the heading error.

    :param q_diff_earth: error quaternion in global coordinates (c.f. calculateErrorQuatEarth), shape (N, 4)
    :return: error in rad, shape (N,)
    """
    return 2 * np.arctan(np.abs(q_diff_earth[:, 3] / q_diff_earth[:, 0]))


def calculateInclinationError(q_diff_earth):
    """
    Calculates the inclination error.

    :param q_diff_earth: error quaternion in global coordinates (c.f. calculateErrorQuatEarth), shape (N, 4)
    :return: error in rad, shape (N,)
    """
    return 2 * np.arccos(np.clip(np.sqrt(q_diff_earth[:, 0] ** 2 + q_diff_earth[:, 3] ** 2), 0, 1))


def calculateRMSE(imu_quat, opt_quat):
    """
    Calculates total/heading/inclination errors in degrees.

    :param imu_quat: IMU orientation, shape (N, 4)
    :param opt_quat: OMC orientation, shape (N, 4)
    :return: dict containing total, heading and inclination errors in degrees
    """

    q_diff_earth = calculateErrorQuatEarth(imu_quat, opt_quat)

    totalError = calculateTotalError(q_diff_earth)
    headingError = calculateHeadingError(q_diff_earth)
    inclError = calculateInclinationError(q_diff_earth)

    return dict(
        total_rmse_deg=np.rad2deg(rmse(totalError)),
        heading_rmse_deg=np.rad2deg(rmse(headingError)),
        inclination_rmse_deg=np.rad2deg(rmse(inclError))
    )

def rmse(diff):
    """Calculates the RMS of the input signal."""
    return np.sqrt(np.nanmean(diff**2))
