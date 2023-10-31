# coding=utf-8



import sys
import os

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from spatialmath.base import *
from math3d.transform import Transform as Trans
import lap_set_pk.lap_set


coordinate_set_path = f'{os.path.dirname(__file__)}/../data/coordinate_set/'
T_0_rcm_file = coordinate_set_path + 'T_0_rcm.csv'
left_rcm_pos_file = coordinate_set_path + 'left_rcm_pos.csv'
right_rcm_pos_file = coordinate_set_path + 'right_rcm_pos.csv'
left_tip_0_file = coordinate_set_path + 'left_tip_0.csv'
right_tip_0_file = coordinate_set_path + 'right_tip_0.csv'

fun_left_0_file = coordinate_set_path + 'fun_left_0.csv'
fun_right_0_file = coordinate_set_path + 'fun_right_0.csv'



def write_file(file_path,data):
    if not os.path.exists(file_path):
        os.mknod(file_path)
    np.savetxt(file_path, data, delimiter=" ")
    print(file_path.replace(coordinate_set_path,'')," write: \n",data,'\n')
    


if __name__ == '__main__':
    
    T_0_rcm = transl(0.56, 0.30, 0.18) @ troty(np.pi)  @ trotz(np.pi)
    
    left_rcm_pos = np.array([0.45108569, 0.26102381, 0.16898061, 1.])
    right_rcm_pos = np.array([0.67721158, 0.25670649, 0.17017894, 1.])
    write_file(T_0_rcm_file,T_0_rcm)
    write_file(left_rcm_pos_file,left_rcm_pos)
    write_file(right_rcm_pos_file,right_rcm_pos)

    # MarkerTwoPose 中用于第一步迭代的初始位置（待确定）
    left_tip_0 = np.array([0.50975898, 0.30078158, 0.05010113])
    right_tip_0 = np.array([0.61339974, 0.31193065, 0.05948734])
    write_file(left_tip_0_file, left_tip_0)
    write_file(right_tip_0_file, right_tip_0)

    # domain_knowledge_optimal_pg 高斯分布
    #高斯分布参数 [       ,               ,               ,           ,   横坐标      ， 纵坐标       ，          ]
    # fun_left = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 473.16764162, 0.00402319]
    # fun_right = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 468.94976255, 0.01276680]
    # fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 600.16764162, 0.00402319]
    # fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 600.94976255, 0.01276680]
    fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 760.18590305, 650.16764162, 0.00402319]
    fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1160.58609198, 650.94976255, 0.01276680]  # 原点在右下角
    write_file(fun_left_0_file, fun_left_0)
    write_file(fun_right_0_file, fun_right_0)

    print(lap_set_pk.lap_set.T_0_rcm)


    






    