
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

data_folder = '/home/irobotcare/桌面/EX_Data/lap/test'

# ---------------------------------------- 视频保存 ----------------------------------------
video_file_path = f'{data_folder}/video/output_video.mp4'                 #视频路径
video_time_stamp_file_path = f'{data_folder}/video/video_timestamp.txt'   #视频每一帧对应时间戳保存路径
video_time_stamp_cvcap_file_path = f'{data_folder}/video/video_timestamp_cvcap_ms.txt'  #两种时间戳

rgb2bgr = False          #true 则执行顺序翻转
video_width = 1920   #并未在所有文件中都采用引用，有些文件中还是用数值设置的分辨率
video_height = 1080
video_fps = 30.0

# ------------------------------------------ 动捕 ------------------------------------------
# qualisys_master_ip = "192.168.253.1"
qualisys_master_ip = "192.168.253.17"
qualisys_password = ''
qtm_rigid_file_path = f'{data_folder}/Qualisys_data/rigid.txt'
rigid_end_record_file_path = f'{data_folder}/Qualisys_data/rigid_end.txt'
rigid_end_calibration_folder = f'{data_folder}/rigid_end_calibration/'

rigid_names={}
rigid_names['qualisys_rob_calibration'] = 'lap'
rigid_names['rcm_calculate'] = 'lap'


# ------------------------------------- rob、camera ip -------------------------------------

# robot_ip = "192.168.100.101"
robot_ip = "192.168.253.10"

camera_usb_id = 0

# -------------------------------- RCM + tip_0 器械末端初始位置 --------------------------------
coordinate_set_folder = f'{data_folder}/coordinate_set'
camera_rcm_pose_file = [f'{coordinate_set_folder}/camera_rcm_pose.csv',\
                        f'{coordinate_set_folder}/camera_rcm_pose1.csv',\
                        f'{coordinate_set_folder}/camera_rcm_pose2.csv',\
                        f'{coordinate_set_folder}/camera_rcm_pose3.csv'  ]
                        # 0: 前下方rcm
                        # 1: 前方1.3m
                        # 2：前方1.4m
                        # 3：左前方                                            
left_rcm_p_file = f'{coordinate_set_folder}/left_rcm_p.csv'
right_rcm_p_file = f'{coordinate_set_folder}/right_rcm_p.csv'


# 标定针
tool_tip=np.array([[0.0],[0.0],[0.105], [1]])
T_rob_tool = np.array([     [-1.0 , 0.0 ,   0.0 ,   -0.0],
                            [ 0.0 , -1.0,   0.0 ,   -0.0],
                            [-0.0, -0.0 ,   1.0 ,   0.105],
                            [ 0.       ,   0.      ,    0.       ,   1.        ]])

# 定位针，快拆版本
rob_needle_end_vector = np.array([0, 0, 0.2])

# 腹腔镜末端到机械臂末端法兰尺寸
rob_lap_end_vector = np.array([0, 0, 0.52])


#2023.09.18 ur5 左臂第一版参数
camera_rcm_pose_index = 0
try:
    T_0_rcm = np.loadtxt(camera_rcm_pose_file[camera_rcm_pose_index],delimiter=',')
except:
    T_0_rcm = np.loadtxt(camera_rcm_pose_file[camera_rcm_pose_index],delimiter=' ')
# if T_0_rcm.shape is not 
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


# ---------------------------------------- 力传感器 ----------------------------------------
'''
T_rob_sensor 只在法兰z轴上有平移，以保证力的计算点和腹腔镜同轴
'''
# Bota Minione
# T_rob_sensor = np.array([   [1,   0,  0,   0   ],
#                             [0,    0,  1,  0   ],
#                             [0,   -1,  0,   0.12],
#                             [0,    0,  0,   1   ]])

#Bota Medusa
T_rob_sensor = np.array([   [0,   1,  0,   0   ],
                            [0,   0,  1,   0   ],
                            [1,   0,  0,   0.12],
                            [0,   0,  0,   1   ]])


def intersection_of_multi_lines(start_points = None, directions = None, input_file = None):  
    """
    计算 n 条 dim 维直线的交点，输入点和方向或存储的文件
    Args:
        start_points (array or list n*dim): 直线上一点坐标
        directions (array or list n*dim): 直线的指向
        input_file (文件路径，其中内容为前面两者 n*2dim)

    Returns:
        m (array dim): 交点坐标（最近点）
    参考: https://zhuanlan.zhihu.com/p/482655943
    """
    print(f'开始计算交点  intersection_of_multi_lines')
    if input_file is not None:
        lap_shaft_arrray = np.loadtxt(input_file,delimiter=',')
        start_points = lap_shaft_arrray[:,:3]
        directions = lap_shaft_arrray[:,3:]

    else:
        start_points = np.array(start_points)
        directions = np.array(directions)

    n, dim = start_points.shape

    G_left = np.tile(np.eye(dim), (n, 1))  
    G_right = np.zeros((dim*n, n))  

    for i in range(n):
        G_right[i*dim:(i+1)*dim, i] = -directions[i, :]

    G = np.concatenate([G_left, G_right], axis=1)  
    d = start_points.reshape((-1, 1)) 

    m = np.linalg.inv(np.dot(G.T, G)).dot(G.T).dot(d)   

    return m[0:dim].squeeze()
    # return m



print('\n==================================================\n\t\t lap_set finished\n==================================================\n')