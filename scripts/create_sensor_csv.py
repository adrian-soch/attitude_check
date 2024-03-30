'''
Read data from https://github.com/dlaidig/broad/tree/main/data_hdf5
And create a .csv file, then multiple methods can operate on that
data to produce quaternion results.

Results can be compared to each other and the ground truth to determine accuracy,
and perfromance.
'''

import scipy.io as sio
import numpy as np
import os
import pandas as pd

csv_save_path = '.' # Path where file will be saved
file_path = os.path.expanduser(
    '~/Downloads/07_undisturbed_fast_rotation_B.mat')

data = sio.loadmat(file_path)
sampling_rate = (float(data["sampling_rate"]))

gt_quat = np.array(data['opt_quat'])
gyro = np.array(data['imu_gyr'])
acc = np.array(data['imu_acc'])
mag = np.array(data['imu_mag'])

total = np.hstack([acc, gyro, mag, gt_quat])

df = pd.DataFrame(total)
df.columns = ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx',
              'my', 'mz', 'q_gt_w', 'q_gt_x', 'q_gt_y', 'q_gt_z']

i = 13249
data = df.iloc[i:i+2000]
print(data)

save_path = os.path.join(csv_save_path, 'sample_sensor_data_hz_' + str(sampling_rate) + '.csv')

print(save_path)
data.to_csv(save_path, index=False)
