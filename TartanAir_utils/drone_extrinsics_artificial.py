import numpy as np
from general_utils.transform_utils import rot2quat

imu_to_cam0 = np.array([[0, -1, 0, 0.1651],
                        [1, 0, 0, -0.1651],
                        [0, 0, 1, 0.0508],
                        [0., 0., 0., 1., ],])

imu_to_cam1 = np.array([[0, -1, 0, -0.1651], 
                        [-1, 0, 0, -0.1651], 
                        [0, 0, -1, -0.0508],
                        [ 0., 0., 0., 1., ]])

imu_to_cam2 = np.array([[ 0, 1, 0, 0.1651],
                        [1, 0, 0, 0],
                        [0, 0, -1, -0.0508],
                        [0., 0., 0., 1., ]])

imu_to_cam3 = np.array([[0, -1, 0, -0.1651], 
                        [-1, 0, 0, 0.1651], 
                        [0, 0, -1, -0.0508],
                        [0., 0., 0., 1., ]])

imu_to_cam4 = np.array([[ 0, -1, 0, 0.1651],
                        [1, 0, 0, 0.1651],
                        [0, 0, 1, 0.0508],
                        [0., 0., 0., 1., ]])

imu_to_cam5 = np.array([[ 0, 1, 0, -0.1651],
                        [-1, 0, 0, 0],
                        [0, 0, 1, 0.0508],
                        [0., 0., 0., 1., ]])


transform_list = [imu_to_cam0, imu_to_cam1, imu_to_cam2, imu_to_cam3, imu_to_cam4, imu_to_cam5]

with open('extrinsics.txt', 'w') as f:
    for i in range(len(transform_list)):
        quat = rot2quat(transform_list[i][:3, :3])
        t = transform_list[i][:3, 3]
        f.write(f'cam{i}:\n')
        f.write(f' "x": {quat[0]},\n "y": {quat[1]},\n "z": {quat[2]},\n "w": {quat[3]}\n')
        f.write(f'  t: [{t[0]}, {t[1]}, {t[2]}]\n')