"""
无论是优化 4dof 还是 3dof 的 lap_X
此程序中收发和计算的都是 4dof
如只优化 3dof, 在 grab_optimal_with_rob.py 中将相应量置0

订阅：
    /cmd/lap_X  由优化程序计算得出的腹腔镜期望位姿(RCM下4自由度)
    /qualisys_rigid_end_json  由动捕系统测量的rigids_end 将其从动捕坐标系转化为机械臂base0 并进行发布

发布：
    /state/lap_X  腹腔镜的实时位姿(RCM下四自由度)
    /base0_rigid_end_pub   base0坐标系下的四自由度
"""
import numpy as np
import torch
from torch import tensor
import torch.optim as optim
import time
import matplotlib.pyplot as plt
import os.path
import sys
import rospy
from std_msgs.msg import Float32MultiArray, String
import math
import json
from collections import deque

sys.path.append(f"{os.path.dirname(__file__)}")
from lap_set_pk import lap_set
from tools import *
sys.path.append("/home/irobotcare/wyh/laparoscope_ws/scripts")
import rokae_basic_fun 
#============================================================================================
rospy.init_node('lap_pose_pub', anonymous=True)
rokae = rokae_basic_fun.rokae()

loop_rate = 100
rob_v_pub_rate = 20
pub_loop_num = int(loop_rate/rob_v_pub_rate)
filter_length = 40 #实时滤波采用的原始数据列表长度
v_cmd_dq = deque( maxlen=filter_length)
for i in range(filter_length):
    v_cmd_dq.append(np.array([0.0,0,0, 0,0,0]))



T_rob_cam = lap_set.T_rob_camera
T_rob_shaft = lap_set.T_rob_shaft
# T_rob_shaft[:3,:3] = np.array([
#     [-1, 0, 0],
#     [0, -1, 0],
#     [0, 0, 1],
# ])
T_0_rcm = lap_set.T_0_rcm
T_0_qualisys = lap_set.T_0_qualisys
T_rcm_0 = T_inv_np(T_0_rcm)
T_shaft_rob = T_inv_np(T_rob_shaft)

T_0_rob_cmd = None
lap_X_cmd = None

T_rcm_shaft_0 = np.array([
    [1, 0,  0,  0],
    [0, 0,  -1, 0],
    [0, 1,  0,  0],
    [0, 0,  0,  1]
])

qualisys_rigid_end_dict={}
base0_rigid_end_dict = {}



def get_lap_X(T_rcm_shaft):
    """
    由 T_rcm_shaf(4*4) 求解 lap_X(4) 
    依赖于固定的
        T_rcm_shaft_0 = np.array([
            [1, 0,  0,  0],
            [0, 0,  -1, 0],
            [0, 1,  0,  0],
            [0, 0,  0,  1]
        ]) 因为解算涉及坐标轴选择和正负
    """
    z_axis = T_rcm_shaft[:3,2]
    x_axis = T_rcm_shaft[:3,0]

    y = -z_axis[2]
    x = np.linalg.norm(z_axis[:2])
    alpha =  math.atan2(y, x)

    y = z_axis[0]
    x = -z_axis[1]
    beta =  math.atan2(y, x)

    x_axis_0gamma = np.array([np.cos(beta), np.sin(beta), 0])
    singamma_vector = np.cross(x_axis_0gamma, x_axis)/np.linalg.norm(x_axis)
    if np.dot(singamma_vector, z_axis) <0:
        gamma = np.arcsin(np.linalg.norm(singamma_vector))
    else:
        gamma = -np.arcsin(np.linalg.norm(singamma_vector))


    d = np.linalg.norm(T_rcm_shaft[:3,3])

    return [alpha, beta, gamma, d]


def T_rcm_shaft_calculate(lap_X):
    """
    根据4自由度计算 T_rcm_cam, 和 grad_optimal中的相同, 但是这个是numpy版本
    Args:
        lap_X (tensor): alpha belta gamma d

    Returns:
        _type_: _description_
    """

    alpha = lap_X[0]
    beta = lap_X[1]
    gamma = lap_X[2]
    d = lap_X[3]


    add_d = np.zeros((4, 4))
    add_d[1, 3] =-d
    T_rcm_cam = rotation_z_T_np(beta) @ rotation_x_T_np(alpha) @  rotation_y_T_np(gamma) @ (add_d + T_rcm_shaft_0)
    return T_rcm_cam

def lap_X_cmd_callback(msg):
    global lap_X_cmd, T_0_rob_cmd
    lap_X_cmd = np.array(msg.data)
    T_rcm_shaft_cmd = T_rcm_shaft_calculate(lap_X_cmd)
    T_0_rob_cmd = T_0_rcm @ T_rcm_shaft_cmd @ T_shaft_rob
    # print(f"T_0_rob_cmd:\n{T_0_rob_cmd}")
    
def qualisys_rigid_end_json_callback(msg):
    global qualisys_rigid_end_dict
    qualisys_rigid_end_dict = json.loads(msg.data)

def rob_desired_speed_calculate(lap_X_now, lap_X_cmd, T_0_rob):
    """
    依赖 T_rcm_shaft0 ，如有修改，需重新判断正负

    Args:
        lap_X_now (_type_): _description_
        lap_X_cmd (_type_): _description_
    """

    lap_X_now = np.array(lap_X_now)
    lap_X_cmd = np.array(lap_X_cmd)


    v_all_max = 0.05 #不包含 v_rmc
    v_rcm_max = 0.03
    lap_X_dif = lap_X_cmd - lap_X_now
    w_k = np.array([0.1, 0.1, 0.1])
    v_z_k = 0.1
    v_rcm_k = 0.5
    #假设构建一个 shaft0 坐标系，z与shaft z同，x保持水平（即shaft0 为 gamma = 0 的shaft）
    #首先计算，在 shaft0 坐标系中，期望的角速度
    lap_X_shaft0 = lap_X_now
    lap_X_shaft0[2] = 0
    T_rcm_shaft0 = T_rcm_shaft_calculate(lap_X_shaft0)
    T_rcm_shaft_now = T_rcm_shaft_calculate(lap_X_now)
    T_shaft_rcm_now = T_inv_np(T_rcm_shaft_now)
    T_shaft_shaft0 = T_shaft_rcm_now @ T_rcm_shaft0 
    w_shaft0 = np.array([0.0,0,0])
    w_shaft0[2] = - w_k[2] * lap_X_dif[2]
    w_shaft0[0] = w_k[0] * lap_X_dif[0]
    w_shaft_beta = -np.array([0, -np.cos(lap_X_now[0]), np.sin(lap_X_now[0])]) * w_k[1] * lap_X_dif[1]
    w_shaft0[:3] = w_shaft0[:3] + w_shaft_beta
    # print(f"w_shaft0: {w_shaft0}")
    w_shaft = T_shaft_shaft0[:3,:3] @ w_shaft0
    # print(f"w_shaft:{w_shaft}")
    w_rob = T_rob_shaft[:3,:3] @ w_shaft
    # print(f"T_rob_shaft: \n{T_rob_shaft}")
    # print(f"w_rob: {w_rob}")
    L = np.linalg.norm(T_rob_shaft[:3,3]) - lap_X_now[3]
    v_rob = np.array([0.0, 0.0, 0.0])
    v_rob[0] = -L * w_rob[1]
    v_rob[1] = L * w_rob[0]
    v_rob[2] = v_z_k * lap_X_dif[3]
    v_rob_norm = np.linalg.norm(v_rob[:3])
    if v_rob_norm > v_all_max:
        limit_k = v_all_max/v_rob_norm
        # print(f"before limit_k: limit_k:{limit_k}, L:{L},\tv_rob:{v_rob}, \tw_rob:{w_rob}")
        w_rob[:2] = w_rob[:2] * limit_k
        v_rob[:3] = v_rob[:3] * limit_k

    # print(f"L:{L},\tv_rob:{v_rob}, \tw_rob:{w_rob}")
    R_0_rob = T_0_rob[:3,:3]
    v_base0 = R_0_rob @ v_rob
    w_base0 = R_0_rob @ w_rob
    v_base0_6dof = np.concatenate((v_base0, w_base0))

    base0_rob_p = T_0_rob[:3,3]
    base0_rcm_p = T_0_rcm[:3,3]
    base0_rob_z = T_0_rob[:3,2]
    base0_vector_rob_rcm = base0_rcm_p - base0_rob_p #rob 指向 rcm
    rcm_error = base0_vector_rob_rcm - np.dot(base0_vector_rob_rcm, base0_rob_z) * base0_rob_z
    rcm_v = v_rcm_k * rcm_error
    rcm_v_norm = np.linalg.norm(rcm_v)
    if rcm_v_norm > v_rcm_max:
        v_rcm_limit_k = v_rcm_max/rcm_v_norm
        rcm_v = v_rcm_limit_k * rcm_v
    # print(f"v6:{v_base0_6dof}")
    v_base0_6dof[:3] = v_base0_6dof[:3] + rcm_v
    return v_base0_6dof

def clean_up():
    rokae.stop()
    print("clean up")

def weighted_moving_average_filter(_before_data, _weights, _stemp_num):
    '''
    加权均值滤波,计算量很小，40步6维力信息滤波耗时约 3e-5 s
    args:
        _before_data(list): _stemp_num * _data_length 每个元素为 _data_length 维力信息

    '''
    _data_length = len(_before_data[0])
    _filtered_data  = _weights.reshape([1, _stemp_num]) @ np.array(_before_data).reshape([_stemp_num, _data_length])
    return _filtered_data.squeeze()


def filter_weight_generate(length=20, sigma=10, miu=0):
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
    # print(f'filter_weight: {_weights}')
    return _weights


filter_weights = filter_weight_generate(filter_length, 20, 0)

if __name__ == "__main__":
    
    

    rospy.Subscriber('/cmd/lap_X',Float32MultiArray, lap_X_cmd_callback)
    rospy.Subscriber('/qualisys_rigid_end_json',String, qualisys_rigid_end_json_callback)

    lap_X_pub = rospy.Publisher('/state/lap_X',Float32MultiArray,queue_size=1)
    lap_X_msg = Float32MultiArray()

    base0_rigid_end_pub = rospy.Publisher('/base0_rigid_end_pub',String,queue_size=1)
    base0_rigid_end_msg = ''


    # temp_now = np.array([0, -0.5,  0, 0.1])
    # temp_cmd = np.array([-0.5, -0.5, 0, 0.2])
    # T_0_rob = rokae.pose.T_matrix()
    # rob_desired_speed_calculate(temp_now, temp_cmd,T_0_rob)
    # exit()


    rate = rospy.Rate(loop_rate)  
    rospy.on_shutdown(clean_up)
    pub_loop_id = 0
    while not rospy.is_shutdown():
        loop_start_time = time.perf_counter()

        T_0_rob = rokae.pose.T_matrix()
        T_0_shaft = T_0_rob @ T_rob_shaft
        T_rcm_shaft = T_rcm_0 @ T_0_shaft
        lap_X_now = get_lap_X(T_rcm_shaft)

        if lap_X_cmd is not None:
            if pub_loop_id >= pub_loop_num:
                v_base0_6dof = rob_desired_speed_calculate(lap_X_now, lap_X_cmd, T_0_rob)
                v_cmd_dq.append(v_base0_6dof)
                v_base0_6dof = weighted_moving_average_filter(v_cmd_dq, filter_weights, filter_length)
                pub_loop_id = -1
                rokae.cv_cmd(v_base0_6dof)
            print(f"lap_X_now:\t{lap_X_now}")
            print(f"lap_X_cmd:\t{lap_X_cmd}")
            print(f"lap_X_step:\t{lap_X_cmd - lap_X_now}")
            print(f"v_base0_6dof:{v_base0_6dof}")

        for key, val in qualisys_rigid_end_dict.items():
            if key in lap_set.model_ex_rigids:
                base0_rigid_end_p = T_0_qualisys @ np.array(val)
                base0_rigid_end_dict.update({key: base0_rigid_end_p.tolist()})
        
        base0_rigid_end_msg = json.dumps(base0_rigid_end_dict)
        base0_rigid_end_pub.publish(base0_rigid_end_msg)


        lap_X_msg.data = lap_X_now
        lap_X_pub.publish(lap_X_msg)
        # print(f"loop time before sleep: {time.perf_counter() - loop_start_time:.6f} s")
        pub_loop_id += 1
        rate.sleep()
        # print(f"loop time after sleep: {time.perf_counter() - loop_start_time:.6f} s")

    




