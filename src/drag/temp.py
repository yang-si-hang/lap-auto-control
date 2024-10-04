import math3d as m3d
import math
import numpy as np
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R


axis = [0,0,1]
radian = math.pi/2
rot_matrix = m3d.Orientation([0,0,radian])
print(rot_matrix)



import os

import time



class mine_pose:
    def __init__(self):
        self.position = self.position_class()
        self.orientation = self.orientation_class()

    class position_class:
        def __init__(self):
            self.x = 0
            self.y = 0
            self.z = 0

        def __str__(self):
            return f"Position:  (x= {self.x:.4f}, \ty= {self.y:.4f}, \tz= {self.z:.4f})"

    class orientation_class:
        def __init__(self):
            self.w = 0
            self.x = 0
            self.y = 0
            self.z = 0
        def __str__(self):
            return f"Orientation:  (w= {self.w:.4f}, \tx= {self.x:.4f}, \ty= {self.y:.4f}, \tz= {self.z:.4f})"
    
    def __str__(self):
        return f"Pose:\n{self.position}\n{self.orientation}"




pose_now = mine_pose()
print(pose_now)
# # 设置滴滴声的频率和时长
# frequency = 1000  # 频率，单位Hz
# duration = 1000  # 时长，单位毫秒
# while True:
#     # os.system('paplay /usr/share/sounds/freedesktop/stereo/complete.oga')
#     os.system('paplay /usr/share/sounds/freedesktop/stereo/suspend-error.oga')
        
j= JointState()
print(j)
print(j.position)


from spatialmath.base import *

x=0.04215641630189087
y = 0.9882752646719122
z = -0.10224511017349353
w = -0.1052652613536581
r1 = R.from_quat([x,y,z,w])
r2 = q2r([w,x,y,z])


print(f'r1:\n{r1.as_matrix()}')
print(f'r2:\n{np.array(r2) - np.array([[-0.974284 , 0.061799 ,-0.216683], [ 0.10485  , 0.975538 ,-0.193217], [ 0.199442 ,-0.210968 ,-0.95693  ]])}')
print(f'{r1.as_quat()}')

def filter_weight_generate( length, sigma, miu=0):
    '''
    用于生成加权均值滤波的权重
    权重符合高斯分布，权重总和为1，最后一个权重最大
    args:
        length: 权重总个数，即滤波长度
        sigma: >0, 高斯分布的标准差，越大则权重越平均
        miu: <= 0, 高斯分布的均值<=0 则保证最后的权重最大，即最新的测量值占比最高
    return:
        _weights: np.array， 向量。
    '''
    _weights = np.zeros([1,length]).squeeze()
    for i in range(length):
        _weights[-1-i] = math.exp(-pow((float(i)-miu),2)/2.0/pow(sigma,2))
    _weights = _weights /np.sum(_weights)
    print(f'filter_weight: {_weights}')
    return _weights
    
filter_weight_generate(20, 10, 0)


matrix = np.array([[11,12,13,14,],[21,22,23,24],[31,31,33,34],[41,42,43,44]])
list = []
list.append(matrix[:3,0].copy())
list.append(matrix[:3,1].copy())
print(list[0])
print(list[1])

def max_norm(matrix, axis = 1):
    # 计算每行向量的模长
    row_norms = np.linalg.norm(matrix, axis=0)
    # 找到模长的最大值
    max_norm = np.max(row_norms)
    return row_norms

print(max_norm(matrix,1))
