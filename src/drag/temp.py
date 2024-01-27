import numpy as np

l =[]
for i in range(10):
    l.append([1,2,3,4,5])
n = np.array(l)
n2 = n.mean(axis=0)
print(n)
print(n2)

import numpy as np

# 创建两个矩阵
matrix1 = np.array([[1, 2], [3, 4]])
matrix2 = np.array([[5, 6]])

# 垂直拼接两个矩阵
result_vertical = np.concatenate((matrix1, matrix2), axis=0)

# 打印结果
print("垂直拼接结果:")
print(result_vertical)

# 创建两个矩阵
matrix3 = np.array([[7], [8]])
matrix4 = np.array([[9], [10]])

# 水平拼接两个矩阵
result_horizontal = np.concatenate((matrix3, matrix4), axis=1)

# 打印结果
print("\n水平拼接结果:")
print(result_horizontal)



from math3d.transform import Transform as Trans

T_rob_sensor = np.array([   [0,    1,  0,   0],
                                    [0,    0,  -1,  0],
                                    [-1,   0,  0,   0],
                                    [0,    0,  0,   1]])

T = Trans(T_rob_sensor)

print(T.array)
print(T.pose_vector)



import os

# 获取当前脚本所在的文件夹路径
current_folder = os.path.dirname(__file__)

print("当前文件夹路径:", current_folder)


list1 = [1,2,3]
list2 = [4,5,6]
print(list1 + list2)
print([list1, list2])

temp_array = np.array([1,2,3,4,5])
print(temp_array - 1)


import os.path
import math

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int8
import urx
import time
import logging
import sys
from spatialmath.base import trotx, troty, trotz, transl, angvec2tr, rpy2tr
from math3d.transform import Transform as Trans
from scipy.spatial.transform import Rotation
import math3d as m3d
import random
from geometry_msgs.msg import TwistStamped

np.set_printoptions(precision=6, suppress=True)

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src")
import drag.force_sensor_receiver as force_sensor_receiver

sys.path.append(f"{os.path.dirname(__file__)}/../../scripts/my_tools")
import key_signal

force_threshold = 1
torque_threshold = 0.2

friction_linear = 2
friction_angular = 0.2

mass = 3  
I_rotation = 0.3025

velocity_linear = np.array([0.0, 0.0, 0.0])
velocity_angular = np.array([0.0, 0.0, 0.0])

damping_linear = 100
damping_angular = 100

acceleration_linear = np.array([0.0, 0.0, 0.0])
acceleration_angular = np.array([0.0, 0.0, 0.0])
acceleration_linear_rate = 0.1
acceleration_angular_rate = 0.1

keyboard_monitor = key_signal.keyboard_monitor_class()

frequency = 500
time_step = 1.0/frequency


force = np.array([12, 234, 25])
torque = np.array([12, 234, 25])

acceleration_linear = (force - friction_linear * velocity_linear/np.linalg.norm(velocity_linear) - damping_linear * velocity_linear)/mass * acceleration_linear_rate
acceleration_angular= (torque - friction_angular * velocity_angular/np.linalg.norm(velocity_angular) - damping_angular * velocity_angular)/I_rotation * acceleration_angular_rate
print(acceleration_linear)
print(acceleration_angular)
print(force * 5)
print(np.linalg.norm(velocity_linear))


