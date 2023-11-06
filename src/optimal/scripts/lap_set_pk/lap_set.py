
import sys
import os

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from spatialmath.base import *
from math3d.transform import Transform as Trans

# sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
# from lap_set_pk import lap_set

# robot_ip = "192.168.100.101"
robot_ip = "192.168.254.10"

camera_usb_id = 0

#2023.09.18 ur5 左臂第一版参数
# T_0_rcm = transl(0.56, 0.30, 0.18) @ troty(np.pi)  @ trotz(np.pi)
T_0_rcm = transl(-0.4504586100287476, 0.0048914174271517, 0.22233258834111135) @ trotx(np.pi)  @ trotz(np.pi/2)
T_0_rcm = transl(-0.5004586100287476, 0.0048914174271517, 0.22233258834111135) @ trotx(np.pi)  @ trotz(np.pi/2)
# print('T_0_rcm:\n',T_0_rcm)
    
# left_rcm_pos = np.array([0.45108569, 0.26102381, 0.16898061, 1.])
left_rcm_pos = np.array([-0.4444386657107944, 0.14681905872550005, 0.17437803674507565, 1.0])
right_rcm_pos = np.array([-0.44256179743616815, -0.07985374184252504, 0.17367135508476444, 1.0])


# MarkerTwoPose 中用于第一步迭代的初始位置（待确定）
# left_tip_0 = np.array([0.50975898, 0.30078158, 0.05010113])
# right_tip_0 = np.array([0.61339974, 0.31193065, 0.05948734])
left_tip_0 = np.array([-0.394386657107944, 0.04681905872550005, 0.1702168250435933])
right_tip_0 = np.array([-0.394386657107944, -0.01985374184252504, 0.16905623665986214])


# domain_knowledge_optimal_pg 高斯分布
#高斯分布参数 [       ,               ,               ,           ,   横坐标      ， 纵坐标       ，          ]
# fun_left = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 473.16764162, 0.00402319]
# fun_right = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 468.94976255, 0.01276680]
# fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 600.16764162, 0.00402319]
# fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 600.94976255, 0.01276680]
# fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 760.18590305, 650.16764162, 0.00402319]
# fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1160.58609198, 650.94976255, 0.01276680]  # 原点在右下角

#2023.09.18 ur5 左臂第一版参数
fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 360.18590305, 350.16764162, 0.00402319]
fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 760.58609198, 350.94976255, 0.01276680]  

fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 310.18590305, 350.16764162, 0.00402319]
fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 810.58609198, 350.94976255, 0.01276680]  


# RobotMove 初始姿态,   两种设置方式
p1 = T_0_rcm @ trotx(np.pi/4) @ troty(-np.pi/10) @ trotz(0.) @ transl(0, 0, 0.02)
pose1 = Trans.get_pose_vector(Trans(p1))
pose1 = Trans.get_pose_vector(Trans(np.array([      [-0.34765125 , 0.79784105 ,-0.49253251 ,-0.55585745],
                                                    [ 0.93746321 , 0.2860465,  -0.19834348 ,-0.02181759],
                                                    [-0.01735937, -0.53068546 ,-0.84739105 , 0.30195085],
                                                    [ 0.       ,   0.      ,    0.       ,   1.        ]])))  #此矩阵为机械臂末端位姿，前面几个是之前用的改过tcp的位姿




RobotMove_init_pose = True

print('lap_setting finished\n')