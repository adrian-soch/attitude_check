import scipy.io as sio
import numpy as np
import pandas as pd

file_path = '/home/adrian/Downloads/07_undisturbed_fast_rotation_B.mat'
# file_path = '/home/adrian/Downloads/01_undisturbed_slow_rotation_A.mat'

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
# df = df.dropna(axis = 0, how = 'any')

i = 13249
data = df.iloc[i:i+5]
df.drop(df.columns[[0]], axis=1, inplace=True)
# df.to_csv('filename.csv', index=False)

csv_string = data.to_csv(sep=',', index=False)
print(csv_string)

# latex_table = data.style.to_latex()
# print(latex_table)
