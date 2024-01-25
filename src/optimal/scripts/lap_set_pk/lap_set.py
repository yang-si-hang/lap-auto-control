
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


# ---------------------------------------- 视频保存 ----------------------------------------
video_file_path = '/home/irobotcare/桌面/EX_Data/lap/test/output_video.avi'                 #视频路径
video_time_stamp_file_path = '/home/irobotcare/桌面/EX_Data/lap/test/video_timestamp.txt'   #视频每一帧对应时间戳保存路径

rgb2bgr = True          #true 则执行顺序翻转
video_width = 1920
video_height = 1080
video_fps = 30.0

# ------------------------------------------ 动捕 ------------------------------------------
qualisys_master_ip = "192.168.253.1"
qualisys_password = ''
qtm_rigid_file_path = '/home/irobotcare/桌面/EX_Data/lap/test/rigid.txt'
rigid_end_record_file_path = '/home/irobotcare/桌面/EX_Data/lap/test/rigid_end.txt'
rigid_end_calibration_file_path = '/home/irobotcare/桌面/EX_Data/lap/test/rigid_end_calibration/'


# ------------------------------------- rob、camera ip -------------------------------------

# robot_ip = "192.168.100.101"
robot_ip = "192.168.253.10"

camera_usb_id = 0

# -------------------------------- RCM + tip_0 器械末端初始位置 --------------------------------
coordinate_set_path = f'{os.path.dirname(__file__)}/../../data/coordinate_set/'
camera_rcm_pose_file = coordinate_set_path + 'camera_rcm_pose.csv'
left_rcm_p_file = coordinate_set_path + 'left_rcm_p.csv'
right_rcm_p_file = coordinate_set_path + 'right_rcm_p.csv'


# 标定针
tool_tip=np.array([[0.0],[0.0],[0.105], [1]])
T_rob_tool = np.array([     [-1.0 , 0.0 ,   0.0 ,   -0.0],
                            [ 0.0 , -1.0,   0.0 ,   -0.0],
                            [-0.0, -0.0 ,   1.0 ,   0.105],
                            [ 0.       ,   0.      ,    0.       ,   1.        ]])


#2023.09.18 ur5 左臂第一版参数
T_0_rcm = np.loadtxt(camera_rcm_pose_file)
# T_0_rcm = transl(-0.4504586100287476, 0.0048914174271517, 0.22233258834111135) @ trotx(np.pi)  @ trotz(np.pi/2)
# T_0_rcm = transl(-0.5004586100287476, 0.0048914174271517, 0.22233258834111135) @ trotx(np.pi)  @ trotz(np.pi/2)
# print('T_0_rcm:\n',T_0_rcm)
    
left_rcm_pos = np.loadtxt(left_rcm_p_file)
right_rcm_pos = np.loadtxt(right_rcm_p_file)
# left_rcm_pos = np.array([-0.4444386657107944, 0.14681905872550005, 0.17437803674507565, 1.0])
# right_rcm_pos = np.array([-0.44256179743616815, -0.07985374184252504, 0.17367135508476444, 1.0])
# print(f'left_rcm_p:\t{left_rcm_pos}')
# print(f'right_rcm_p:\t{right_rcm_pos}')
# print(f'camera_rcm_pose:\n{T_0_rcm}')


# MarkerTwoPose 中用于第一步迭代的初始位置（待确定）
left_tip_0 = ((left_rcm_pos + right_rcm_pos)/2)[:-1] +[0, 0, -0.05]
right_tip_0 = ((left_rcm_pos + right_rcm_pos)/2)[:-1] +[0, 0, -0.05]
# left_tip_0 = np.array([-0.394386657107944, 0.04681905872550005, 0.1702168250435933])
# right_tip_0 = np.array([-0.394386657107944, -0.01985374184252504, 0.16905623665986214])
# print(left_tip_0)

# ------------------------------------------------------------------------------------------


# domain_knowledge_optimal_pg 高斯分布
#高斯分布参数 [       ,               ,               ,           ,   横坐标      ， 纵坐标       ，          ]
# fun_left = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 473.16764162, 0.00402319]
# fun_right = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 468.94976255, 0.01276680]
# fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 600.16764162, 0.00402319]
# fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 600.94976255, 0.01276680]
# fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 760.18590305, 650.16764162, 0.00402319]
# fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1160.58609198, 650.94976255, 0.01276680]  # 原点在右下角

#2023.09.18 ur5 左臂第一版参数
#高斯分布参数 [       ,               ,               ,           ,  横坐标(向右为正)， 纵坐标（向下为正），          ]
# fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 360.18590305, 350.16764162, 0.00402319]
# fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 760.58609198, 350.94976255, 0.01276680]  

fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 400.18590305, 580.16764162, 0.00402319]
fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1300.58609198, 580.94976255, 0.01276680]  


# RobotMove 初始姿态,   两种设置方式
RobotMove_init_pose = False

p1 = T_0_rcm @ trotx(np.pi/4) @ troty(-np.pi/10) @ trotz(0.) @ transl(0, 0, 0.02)
pose1 = Trans.get_pose_vector(Trans(p1))
pose1 = Trans.get_pose_vector(Trans(np.array(   [[-0.9048253684647052, -0.3879666119058063, 0.17542223527546316, -0.007310765609089019],
                                                [-0.4074949172898341, 0.6696008859295312, -0.6209529337601507, -0.7397633136266437],
                                                [0.12344612171170741, -0.6333376363431407, -0.7639662907652549, 0.2657086965460797],
                                                [0.0, 0.0, 0.0, 1.0]])))  #此矩阵为机械臂末端位姿，前面几个是之前用的改过tcp的位姿


print('\n==================================================\n\t\t lap_set finished\n==================================================\n')