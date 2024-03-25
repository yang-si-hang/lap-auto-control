'''
用于根据临时保存的相机标定结果：
    运算结果临时保存在 ../data/Camera_Calibration/mtx_temp.csv 和 camera_tool_temp.csv
    如果确认使用计算结果，请copy到 ../data/Camera_Calibration/camera_tool.csv （相对末端的坐标）和 mtx.csv （内参）

计算shaft的变换矩阵来判断同轴度
'''

import os.path

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import urx
import time
import logging
import sys
from spatialmath.base import trotx, troty, trotz, transl, angvec2tr, rpy2tr
from math3d.transform import Transform as Trans
sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

path = os.path.dirname(__file__)


q0 = np.pi/6

# left_base = np.array([[-0.9981, 0.06149, -0.004677, 0.01738],
#                       [0.04068, 0.7135, 0.6995, 0.2161],
#                       [0.04635, 0.698, -0.7146, 1.534],
#                       [0, 0, 0, 1]])
# left_base = left_base @ trotz(-np.pi)
# left_base_inv = np.linalg.inv(left_base)
# right_base = transl(0.0005, -0.22663, 1.53885) @ rpy2tr(2.3446, 0.0426, -0.0476)
# right_base = right_base @ trotz(-np.pi)
right_base = np.identity(4)

right_base_inv = np.linalg.inv(right_base)

camera_tcp = np.loadtxt(f'{path}/../data/Camera_Calibration/camera_tool_temp.csv')

shaft_tcp = camera_tcp @ trotx(q0) #将坐标系转回shaft

print(f'shaft:\n{shaft_tcp}')
