import scipy.io as sio
import numpy as np
import pandas as pd

file_path = '/home/adrian/Downloads/07_undisturbed_fast_rotation_B.mat'

# Load data from 'file.mat'
data = sio.loadmat(file_path)

# Print the loaded data
# print(data)

gt_quat = np.array(data['opt_quat'])
gyro = np.array(data['imu_gyr'])
acc = np.array(data['imu_acc'])
mag = np.array(data['imu_mag'])

total = np.hstack([acc, gyro, mag, gt_quat])

df = pd.DataFrame(total)
df = df.dropna(axis = 0, how = 'any')

head = df.head(10)
latex_table = head.style.to_latex()
print(latex_table)