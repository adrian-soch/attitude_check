'''
This script will use the python AHRS (https://ahrs.readthedocs.io/en/latest/)
as a psuedo oracle for generating test cases.
'''

from ahrs.common.orientation import acc2q, am2angles, ecompass, rpy2q
from ahrs.filters import Madgwick
import numpy as np


def generate_acc2q_tests(acc):
    # acc =
    quat = acc2q(a=acc)

    print(f'In: {acc} Out:', end=" ")
    for q in quat:
        print(f'{q:.9f}', end=" ")
    print("\n")


def generate_ecompass_tests(acc, mag):
    quat = ecompass(acc, mag, frame='NED', representation='quaternion')

    print(f'Acc: {acc}, Mag: {mag} Out:', end=" ")
    for q in quat:
        print(f'{q:.9f}', end=" ")
    print("\n")


def generate_am2q_tests(acc, mag):
    print(f'Acc: {acc}, Mag: {mag} Out:', end=" ")

    rpy = am2angles(acc, mag)
    print(np.flip(rpy))
    quat = rpy2q(np.flip(rpy))

    print(quat)


am2angles


def get_mag_to_quat_cases():

    generate_am2q_tests(np.array(
        [-6.382152, -7.969839, 0.099505]), np.array([36.700461, 22.613240, 8.881961]))

    generate_am2q_tests(np.array([0.0, 0.0, 9.81]),
                        np.array([16676.8, -3050.9, 49916.9]))


def get_acc_to_quat_cases():

    generate_acc2q_tests([0.090941, -0.031273, 9.759028])

    # gt = #  0.802582 & 0.209335 & 0.088698 & 0.551520
    generate_acc2q_tests([0.283217, 2.540909, 7.708738])

    # -0.093491 & -0.892442 & 0.090310 & 0.432031 \\
    generate_acc2q_tests([7.550523, 2.391483, -0.958994])

    generate_acc2q_tests([-9.81, 0, 0])


def get_marg_estimate_cases():

    q0 = [0.49666186227,0.034292027603,-0.0510015643620,0.86576549471]

    q_actual = [[0.49593818106, 0.0326998015547, -0.0510947167457, 0.86623632656],
                [0.49519980631, 0.0312979556660, -0.0513729764280, 0.86669395238],
                [0.49443437746, 0.0302718565058, -0.0520166472497, 0.86712890015],
                [0.49366783997, 0.029245689468, -0.0526602014369, 0.867561903615]]

    gyr = np.array([[-0.5006895838366212, 0.8074957770569486, 0.5528888911052677],
           [-0.5326483077698894, 0.6743323911175373, 0.5262569120490862],
           [-0.5624759846864724, 0.5560845889656695, 0.5166698184678814],
           [-0.5880433128989375, 0.39415992228264346, 0.5017542346803379]], dtype=np.float32)
    acc = np.array([[0.47451228800383594, 1.0183972122654374, 8.461165125378162],
           [0.3783742880038359, 0.5534032122654374, 8.264965125378161],
           [0.27340728800383585, 0.05015021226543746, 8.015791125378161],
           [0.3067612880038358, -0.2853517877345626, 8.159017125378163]], dtype=np.float32)
    mag = np.array([[10.04650728048766, -6.088486551348383, -43.828022623283225],
           [11.084734479270914, -6.77198980959856, -44.35222010806493],
           [10.19482545174241, -6.923879422543039, -43.97779333322085],
           [9.30491642421391, -7.075769035487526, -43.60336655837678]], dtype=np.float32)
    est = Madgwick(gyr=gyr, acc=acc, mag=mag, frequency=285.71428571, q0=q0, gain_imu=0.033, gain_marg=0.041)
    print(est.Q)

def get_imu_estimate_cases():

    q0 = [0.49666186227,0.034292027603,-0.0510015643620,0.86576549471]

    q_actual = [[0.49593818106, 0.0326998015547, -0.0510947167457, 0.86623632656],
                [0.49519980631, 0.0312979556660, -0.0513729764280, 0.86669395238],
                [0.49443437746, 0.0302718565058, -0.0520166472497, 0.86712890015],
                [0.49366783997, 0.029245689468, -0.0526602014369, 0.867561903615]]

    gyr = np.array([[-0.5006895838366212, 0.8074957770569486, 0.5528888911052677],
           [-0.5326483077698894, 0.6743323911175373, 0.5262569120490862],
           [-0.5624759846864724, 0.5560845889656695, 0.5166698184678814],
           [-0.5880433128989375, 0.39415992228264346, 0.5017542346803379]], dtype=np.float32)
    acc = np.array([[0.47451228800383594, 1.0183972122654374, 8.461165125378162],
           [0.3783742880038359, 0.5534032122654374, 8.264965125378161],
           [0.27340728800383585, 0.05015021226543746, 8.015791125378161],
           [0.3067612880038358, -0.2853517877345626, 8.159017125378163]], dtype=np.float32)
    est = Madgwick(gyr=gyr, acc=acc, frequency=285.71428571, q0=q0, gain_imu=0.033, gain_marg=0.041)
    print(est.Q)


def main():
    # get_acc_to_quat_cases()
    # get_mag_to_quat_cases()
    get_marg_estimate_cases()
    get_imu_estimate_cases()


if __name__ == "__main__":
    main()
