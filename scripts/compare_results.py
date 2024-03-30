'''
Compute accuracy metrics relative to the data and ground truth created by
    `create_sensor_csv.py`.

References: https://github.com/dlaidig/broad/blob/main/example_code/broad_utils.py
            https://github.com/Mayitzin/ahrs
'''
import numpy as np

from rmse_orientation_utils import *


def get_ahrs_results(data, freq, no_mag=False):
    '''
    Use the madwick implementation from https://github.com/Mayitzin/ahrs
    as a benchmark
    '''
    from ahrs.filters import Madgwick

    filt = Madgwick(frequency=freq, gain_imu=0.033, gain_marg=0.041)

    res = []
    q_prev = np.array([1.0, 0.0, 0.0, 0.0])
    for measurements in data:
        acc = measurements[0:3]
        gyr = measurements[3:6]
        mag = measurements[6:9]

        if no_mag:
            q = filt.updateIMU(q=q_prev, acc=acc, gyr=gyr)
        else:
            q = filt.updateMARG(q=q_prev, acc=acc, gyr=gyr, mag=mag)
        res.append(q.tolist())
        q_prev = q

    return np.array(res)


def read_csv_to_array(file_path, skip_header=0):
    """read in a csv file.

    Args:
        file_path (string): Path to csv file
        skip_header (int, optional): Number of rows to skip. Defaults to 0.

    Returns:
        numpy ndarray
    """
    return np.genfromtxt(file_path, delimiter=',', skip_header=skip_header)


def main():

    data = '../docs/sample_sensor_data_hz_285.7142857142857.csv'
    data = read_csv_to_array(data, skip_header=1)

    # Compute quats using the AHRS python package
    ahrs_marg_res = get_ahrs_results(data, freq=285.7142857142857)
    ahrs_imu_res = get_ahrs_results(data, freq=285.7142857142857, no_mag=True)

    # Calc Attitude Check MARG error relative to GT
    ac_res = read_csv_to_array('../docs/attitude_check_marg_results.csv')
    final_results_ac = calculateRMSE(ac_res, data[:, 9:14])
    print("\nAC MARG Results: \n")
    print(final_results_ac)

    # Calc Attitude Check IMU error relative to GT
    ac_res = read_csv_to_array('../docs/attitude_check_imu_results.csv')
    final_results_ac = calculateRMSE(ac_res, data[:, 9:14])
    print("\nAC IMU Results: \n")
    print(final_results_ac)

    # Calc AHRS MARG error relative to GT
    final_results_ahrs = calculateRMSE(ahrs_marg_res, data[:, 9:14])
    print("\nAHRS MARG Results: \n")
    print(final_results_ahrs)

    # Calc AHRS IMU error relative to GT
    final_results_ahrs = calculateRMSE(ahrs_imu_res, data[:, 9:14])
    print("\nAHRS IMU Results: \n")
    print(final_results_ahrs)


if __name__ == "__main__":
    main()
