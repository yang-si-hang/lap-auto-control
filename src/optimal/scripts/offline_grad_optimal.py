"""
用于离线数据分析
考察姿态优化相关算法
基于 grad_optimal_with_rob.py(部分复制至本程序并修改)
注意在 main 中设置处理第几帧数据
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
import json
from std_msgs.msg import Float32MultiArray,String
import math


sys.path.append(f"{os.path.dirname(__file__)}")
from lap_set_pk import lap_set
from tools import *
#============================================================================================
LAP_X_DOF = 3 #alpha、beta、gamma、d 4自由度，如果设为3，则不优化 gamma = 0
draw_on = True #是否绘图并保存
show_on = True #是否显示图片
draw_and_save_step = 5 #优化几次存储一次优化图像 （一次优化图像包含完整的多次迭代）
draw_and_save_id = 0

folder = f"/home/irobotcare/桌面/EX_Data/lap/ME_Auto/fenghe2"
lap_pose_pub_cmd_file = f"{folder}/auto_model_ex/lap_pose_pub_cmd.json"
grad_opt_lap_X_file = f"{folder}/auto_model_ex/grad_opt_lap_X.txt"
rigid_end_file = f"{folder}/Qualisys_data/rigid_end.txt"
# optimal_fig_folder = f"{folder}/optimal_fig"
optimal_offline_folder = f"{folder}/optimal_offline"
if not os.path.exists(optimal_offline_folder): os.makedirs(optimal_offline_folder)

lap_X_record_path = f"{folder}/grad_opt_lap_X.txt"

lap_pose_pub_cmd = None
rigid_end = None
grad_opt_lap_X = None




alpha_max = -15 *np.pi/180
alpha_min = -50 *np.pi/180
beta_max = 50 *np.pi/180
beta_min = -50 *np.pi/180
d_max = 0.20
d_min = 0.05



out_of_view_k = tensor([0.02, 0.02])

img_width = lap_set.video_width
img_height = lap_set.video_height

base0_rigid_end_dict = {}

cam_K = tensor([
    [1.321383458370769176e+03,  0,                          9.680885724568087198e+02,   0],
    [0,                         1.323800731972558879e+03,   5.572575084185185688e+02,   0],
    [0,                         0,                          1,                          0]
])
cam_K[:3,:3] = torch.tensor(lap_set.camera_K)
cam_dist = torch.tensor(lap_set.camera_dist)

#-------------------------------------------------------------------
phases_list = ["gripping", "needling", "tightening", "knotting", "cutting", "moving", "grabbing", "placing"]

inst_in_phase={ #每个阶段使用的器械
    "gripping": ['fenliqian','chizhenqi'],
    "needling": ['fenliqian','chizhenqi'],
    "tightening": ['fenliqian','chizhenqi'],
    "knotting": ['fenliqian','chizhenqi'],
    "cutting": ['fenliqian','jiandao'],
    "moving": ['changqian'],
    "grabbing": ['changqian'],
    "placing": ['changqian']
}
inst_changing = False #用于一些阶段切换需要换器械时进行延迟
#-------------------------------------------------------------------
pixel_adjust_dict={
    "gripping": torch.tensor([60, 70, 0]),
    "needling": torch.tensor([-50, 200, 0]),
    "tightening": torch.tensor([150, 0, 0]),
    "knotting": torch.tensor([10, 30, 0]),
    "cutting": torch.tensor([20, 130, 0]),
    "moving": torch.tensor([250, 0, 0]),
    "grabbing": torch.tensor([200, 100, 0]),
    "placing": torch.tensor([250, 70, 0])
}
#--------------------------- 二维像素分布 （生成函数、参数） ---------------------------------
class Gaussian_Distribution_2D_tensor():
    def __init__(self, func_args):
        """
        Args:
            func_args (list_like): 高斯参数：[幅度, 旋转角度, x标准差, y标准差, x中心, y中心, 常数偏移]
        """
        # 将func_args转换为torch.tensor，并确保可以计算梯度
        self.func_args = torch.tensor(func_args, dtype=torch.float32, requires_grad=False)
    
    def val(self, x_y=None, x=None, y=None):
        """
        计算给定坐标下高斯分布的值
        Args:
            (x_y 或 x和y 2选1)
            x_y (list_like): [0]为x, [1]为y. Defaults to None.
            x (float): Defaults to None.
            y (float): Defaults to None.
        Returns:
            val(float)
        """
        if x_y is not None:
            x = x_y[0]
            y = x_y[1]

        exponent = -(((x - self.func_args[4]) * torch.cos(self.func_args[1] * torch.pi / 180) + (y - self.func_args[5]) * torch.sin(self.func_args[1] * torch.pi / 180)) / self.func_args[2]) ** 2\
                - ((-(x - self.func_args[4]) * torch.sin(self.func_args[1] * torch.pi / 180) + (y - self.func_args[5]) * torch.cos(self.func_args[1] * torch.pi / 180)) / self.func_args[3]) ** 2
        val = (
            self.func_args[6]
            + self.func_args[0] * torch.exp(exponent)
        )
        # print(f"Gaussian exponent: {exponent}, val: {val}")
        return val

'''
f_0 是一个尺度参数，影响整个函数的幅度。
f_1 是旋转角度参数，表示二维高斯的旋转。
f_2 和 f_3 是两个标准差参数，控制高斯分布的形状。
f_4 和 f_5 是二维高斯分布的中心坐标。
f_6 是一个常数偏移项。
'''
'''                  幅度，       旋转角度，   x标准差,        y标准差,       x中心,       y中心,         常数偏移  '''
amplitude = 10  #统一幅值
sigma_x_k = 6   #扩大sigma
sigma_y_k = 6
# fun_left_0 = [5.410334, 137.06098690, 377.40041913*sigma_x_k, 431.40073990*sigma_y_k, 400.18590305, 580.16764162, 0.00402319]
# fun_right_0 = [7.491880, 96.42623581, 99.32096722, 254.80939073, 1300.58609198, 580.94976255, 0.01276680] 
# fun_right_0 = [7.491880, 96.42623581, 299.32096722*sigma_x_k, 354.80939073*sigma_y_k, 1300.58609198, 580.94976255, 0.01276680] 
# fun_left_0 = [5.410334, 137.06098690, 1377.40041913, 1431.40073990, 400.18590305, 580.16764162, 0.00402319]
# fun_right_0 = [7.491880, 96.42623581, 1199.32096722, 1254.80939073, 1300.58609198, 580.94976255, 0.01276680]
# gauss_left = Gaussian_Distribution_2D_tensor(fun_left_0)
# gauss_right = Gaussian_Distribution_2D_tensor(fun_right_0)
gauss_2d_dist_dict={}
for phase, inst_list in inst_in_phase.items():
    gauss_2d_dist_dict[phase] = {}

'''                                     幅度，          旋转角度，             x标准差,             y标准差,            x中心,              y中心,              常数偏移  '''
# gauss_2d_params_dict_origin = {
#     "gripping": {'fenliqian':[0.29679336028228626, -24297.291039090396, 249.0925396162229, 159.19836126414364, 826.3540707773755, 611.6977443500055, 0.0014353834870833061],
#                  'chizhenqi':[0.23936022047581867, -2645090.74979495, -275.4520573495296, -180.2748809298906, 1069.3232539146484, 463.3722690468168, 0.0010382687460151153]},
#     "needling": {'fenliqian':[0.1596771363272815047, 102.8059372243528884, 128.6938381590713334, 217.4415386944029365, 846.4356659584245790, 727.6013040538824725, 0.001731287314451205933],
#                  'chizhenqi':[0.10123281354321009, 275932.51416596415, 256.28881889520864, 168.04463194794437, 1092.4134437266152, 533.7160824913244, 0.0011719898444434685]},
#     "tightening": {'fenliqian':[0.1641001447875553, -401038.64755309053, 331.5054929131512, 169.39156182042936, 749.2979762710125, 549.222899068901, 0.0018633787942904038],
#                    'chizhenqi':[0.188852570404634, 1491918.7581617485, 163.1901344595184, 305.8117482461911, 1241.7516516727053, 406.8656641384972, 0.0010562624900361495]},
#     "knotting": {'fenliqian':[0.2438089602015827, 377111.42279701825, 272.73174593253003, 208.47757430838297, 738.5186343154119, 535.0408307344952, 0.0038910930170683294],
#                  'chizhenqi':[0.23518751762607656, -323547.36954917957, 213.7419013205909, 303.5886934024331, 1023.2503932472855, 527.7378733139883, -0.00025109612540995176]},
#     "cutting": {'fenliqian':[0.03017085076602141, -33284.27457708106, 237.7218701150215, 119.22748231994004, 683.2750443514196, 593.3231543199093, 0.000511954513715334],
#                 'jiandao':[0.028992289693481492, 1465903.7964912858, -194.44442524935735, -159.00062660636607, 1153.3906376084353, 597.0727883756015, 0.0003419666337863225]},
#     "moving": {'changqian':[0.08377206513361535, 2721870.014681193, 160.7254867501931, 212.91399514744307, 808.7535341529272, 508.3785172533859, 0.00025652060477846796]},
#     "grabbing": {'changqian':[0.05195384944849729, 633079.7095369393, 210.0889701371155, 157.3175168121141, 782.4599134274677, 520.8383984189799, 0.0001251851142344139]},
#     "placing": {'changqian':[0.07144402610804718, -894543.4418581283, -144.57888274771312, -148.59171371369692, 869.5551032790868, 578.8806952651478, 0.00011821834733712267]}
# }
'''                                     幅度，          旋转角度，             x标准差,             y标准差,            x中心,              y中心,              常数偏移  '''
gauss_2d_params_dict = {
    'gripping': {'fenliqian': [0.29679336028228626, -177.29103909039623, 249.0925396162229, 159.19836126414364, 826.3540707773755, 611.6977443500055, 0.0014353834870833061],
                 'chizhenqi': [0.23936022047581867, -170.74979494977742, -275.4520573495296, -180.2748809298906, 1069.3232539146484, 463.3722690468168, 0.0010382687460151153]},
    'needling': {'fenliqian': [0.1596771363272815, 102.80593722435289, 128.69383815907133, 217.44153869440294, 846.4356659584246, 727.6013040538825, 0.001731287314451206],
                 'chizhenqi': [0.10123281354321009, 172.5141659641522, 256.28881889520864, 168.04463194794437, 1092.4134437266152, 533.7160824913244, 0.0011719898444434685]}, 
    'tightening': {'fenliqian': [0.1641001447875553, -358.64755309053, 331.5054929131512, 169.39156182042936, 749.2979762710125, 549.222899068901, 0.0018633787942904038],
                   'chizhenqi': [0.188852570404634, 78.75816174852662, 163.1901344595184, 305.8117482461911, 1241.7516516727053, 406.8656641384972, 0.0010562624900361495]},
    'knotting': {'fenliqian': [0.2438089602015827, 191.42279701825464, 272.73174593253003, 208.47757430838297, 738.5186343154119, 535.0408307344952, 0.0038910930170683294],
                 'chizhenqi': [0.23518751762607656, -267.3695491795661, 213.7419013205909, 303.5886934024331, 1023.2503932472855, 527.7378733139883, -0.00025109612540995176]},
    'cutting': {'fenliqian': [0.03017085076602141, -164.27457708105794, 237.7218701150215, 119.22748231994004, 683.2750443514196, 593.3231543199093, 0.000511954513715334],
                'jiandao': [0.028992289693481492, 343.79649128578603, -194.44442524935735, -159.00062660636607, 1153.3906376084353, 597.0727883756015, 0.0003419666337863225]},
    'moving': {'changqian': [0.08377206513361535, 270.0146811930463, 160.7254867501931, 212.91399514744307, 808.7535341529272, 508.3785172533859, 0.00025652060477846796]},
    'grabbing': {'changqian': [0.05195384944849729, 199.7095369392773, 210.0889701371155, 157.3175168121141, 782.4599134274677, 520.8383984189799, 0.0001251851142344139]},
    'placing': {'changqian': [0.07144402610804718, -303.4418581282953, -144.57888274771312, -148.59171371369692, 869.5551032790868, 578.8806952651478, 0.00011821834733712267]}}
#统一幅值，扩大标准差
for phase, insts_dict in gauss_2d_params_dict.items():
    for inst, val in insts_dict.items():
        gauss_2d_params_dict[phase][inst][0] = amplitude
        gauss_2d_params_dict[phase][inst][2] = sigma_x_k * gauss_2d_params_dict[phase][inst][2]
        gauss_2d_params_dict[phase][inst][3] = sigma_y_k * gauss_2d_params_dict[phase][inst][3]
        gauss_2d_dist_dict[phase][inst] = Gaussian_Distribution_2D_tensor(gauss_2d_params_dict[phase][inst])
        # print(f"{phase}\t{inst}\tsigma_xy:{math.sqrt(gauss_2d_params_dict[phase][inst][2]**2 + gauss_2d_params_dict[phase][inst][3]**2)}")

# for phase, insts_dict in gauss_2d_dist_dict.items():
#     print(f"phase ----------")
#     for inst, val in insts_dict.items():
#         print(inst)

# gauss_dict_temp = gauss_2d_params_dict.copy()
# for phase, insts_dict in gauss_2d_params_dict.items():
#     for inst, val in insts_dict.items():
#         angle = gauss_dict_temp[phase][inst][1]
#         roll_num = int(angle/360)
#         angle = angle - 360* roll_num
#         gauss_dict_temp[phase][inst][1] = angle
# print(gauss_dict_temp)

#------------------------ 深度分布 ------------------------------
# lap d 高斯拟合结果
# [gripping]      mean:   0.15950481412772527     std:0.026330795983790005
# [needling]      mean:   0.1859067586628832      std:0.021118937615869898
# [tightening]    mean:   0.13961117283683613     std:0.027377602051642407
# [knotting]      mean:   0.16773371255903682     std:0.02228479439562573
# [cutting]       mean:   0.15678583345909003     std:0.02935302065758646
# [moving]        mean:   0.11960882171313425     std:0.03450422444146625
# [grabbing]      mean:   0.17587754305798572     std:0.03316702150082586
# [placing]       mean:   0.15800867650553185     std:0.03952393029223555
# 定义正态分布的均值(mu)和标准差(sigma)
d_mu = {
    "gripping": torch.tensor(0.15950481412772527),
    "needling": torch.tensor(0.1859067586628832 + 0.01),
    "tightening": torch.tensor(0.13961117283683613),
    "knotting": torch.tensor(0.16773371255903682),
    "cutting": torch.tensor(0.15678583345909003),
    "moving": torch.tensor(0.11960882171313425),
    "grabbing": torch.tensor(0.17587754305798572+0.022),
    "placing": torch.tensor(0.15800867650553185+0.035)
}
d_sigma = {
    "gripping": torch.tensor(0.026330795983790005),
    "needling": torch.tensor(0.021118937615869898),
    "tightening": torch.tensor(0.027377602051642407),
    "knotting": torch.tensor(0.02228479439562573),
    "cutting": torch.tensor(0.02935302065758646),
    "moving": torch.tensor(0.03450422444146625),
    "grabbing": torch.tensor(0.03316702150082586),
    "placing": torch.tensor(0.03952393029223555)
}
d_normal_distributions = {} #lap_X 中 d 在不同 phase 下的正态分布
for _phase in phases_list:
    d_normal_distributions[_phase] = torch.distributions.Normal(d_mu[_phase], d_sigma[_phase])
#-------------------------------------------------------------------
phase = "gripping"

if LAP_X_DOF == 4:
    # 初始化优化参数（x）,用于优化，和发布控制指令，作为desire
    lap_X = torch.tensor([-20.0/180.0*torch.pi, 0.0, 0.0, 0.15], requires_grad= True)  # 初始值为0，并启用梯度计算
if LAP_X_DOF == 3:
    lap_X = torch.tensor([-20.0/180.0*torch.pi, 0.0, 0.15], requires_grad= True)  # 初始值为0，并启用梯度计算
lap_X_now = None #接收到的现在机器人的 lap_X 实际值



# inst_0_p1 = tensor([-0.02, 0.21, 0.03, 1]) #0坐标系下器械位置
# inst_0_p2 = tensor([0.02, 0.21, 0.03, 1])
# inst_0_p1 = None #0坐标系下器械位置
# inst_0_p2 = None
inst_0_p_dict = {
    'fenliqian':None,
    'chizhenqi':None,
    'jiandao':None,
    'changqian':None
}


Camera_Calibration_folder = f'{lap_set.data_folder}/Camera_Calibration'
# T_rob_camera = np.loadtxt(f"{Camera_Calibration_folder}/camera_tool.csv")
T_rob_camera = torch.tensor(lap_set.T_rob_camera)




T_0_rcm = tensor(lap_set.T_0_rcm, dtype= torch.float32)
T_0_qualisys = lap_set.T_0_qualisys

class instrument():
    def __init__(self, name):
        self.name = name


# T_shaft_cam = rotation_x_T(tensor(-torch.pi/6))
T_shaft_cam = torch.tensor(lap_set.T_shaft_camera, dtype=torch.float32)
T_rcm_shaft_0 = tensor([
    [1, 0,  0,  0],
    [0, 0,  -1, 0],
    [0, 1,  0,  0],
    [0, 0,  0,  1]
], dtype= torch.float32)



def T_rcm_shaft_calculate(lap_X):
    """
    根据4自由度计算 T_rcm_cam
    Args:
        lap_X (tensor): alpha belta gamma d

    Returns:
        _type_: _description_
    """
    if LAP_X_DOF == 4:
        alpha = lap_X[0]
        beta = lap_X[1]
        gamma = lap_X[2]
        d = lap_X[3]
    else:
        alpha = lap_X[0]
        beta = lap_X[1]
        gamma = torch.tensor(0.0)
        d = lap_X[2]

    add_d = torch.zeros(4, 4)
    add_d[1, 3] =-d
    T_rcm_shaft = rotation_z_T(beta) @ rotation_x_T(alpha) @  rotation_y_T(gamma) @ (add_d + T_rcm_shaft_0)
    return T_rcm_shaft


def apply_distortion(pixel_p):
    """根据畸变参数和未畸变投影坐标，计算畸变后的投影坐标
    Args:
        pixel_p : 畸变前的像素坐标
    Returns:
        畸变后的像素坐标
    """
    global cam_dist
    u = pixel_p[0]/img_width
    v = pixel_p[1]/img_height
    # 计算径向畸变
    r_sq = u**2 + v**2

    # 径向畸变
    delta_u = img_width * (u * (cam_dist[0] * r_sq + cam_dist[1] * torch.pow(r_sq, 2) + cam_dist[4] * torch.pow(r_sq,3)) + 2 * cam_dist[2] * u * v + cam_dist[3] * (r_sq + 2 * torch.pow(u,2)))
    delta_v = img_height * (v * (cam_dist[0] * r_sq + cam_dist[1] * torch.pow(r_sq, 2) + cam_dist[4] * torch.pow(r_sq,3)) + cam_dist[2] * (r_sq + 2 * torch.pow(v,2)) + 2*cam_dist[3] * u * v)
    # print(f"delta_uv:{delta_u.detach().numpy(), delta_v.detach().numpy()}")
    # 加入畸变
    pixel_p[0] += delta_u
    pixel_p[1] += delta_v

    return pixel_p


def pixel_position(T_0_cam, inst_0_p):
    """
    计算器械末端投影的二维坐标

    Args:
        T_0_cam (4*4): 0 坐标系下 镜头坐标系(已旋转30°)
        inst_0_p (4): 0 坐标系下 器械末端齐次坐标 
        apply_distortion_bool: 是否施加畸变

    Returns:
        pixel_p(3): 像素齐次坐标[u, v, 1]
    """
    T_cam_0 = T_inv(T_0_cam)
    cam_inst_p = T_cam_0 @ inst_0_p
    z = cam_inst_p[2]
    pixel_p = cam_K @ cam_inst_p/z
    # return pixel_p
    distort_pixel_p = apply_distortion(pixel_p)
    return distort_pixel_p






def pixel_loss(pixel_p,distribution, weight = 1):
    """
    计算单个器械的损失函数

    Args:
        pixel_p (3): 2d视野中器械末端齐次像素坐标
        distribution : 分布,包含val([x,y])方法
        weight (float): 权重 Defaults to 1.

    Returns:
        float: 损失函数(越小越好)
    """
    return - weight * distribution.val(pixel_p[:2])


def out_of_view_loss(pixel_p):
    """
    器械超出视野的loss
    Args:
        pixel_p (3): 2d视野中器械末端齐次像素坐标
    """
    global out_of_view_k, img_width, img_height
    x= pixel_p[0]
    y= pixel_p[1]
    loss = 0
    if x <0:
        loss += (0-x)*out_of_view_k[0]
    elif x > img_width:
        loss += (x-img_width)*out_of_view_k[0]

    if y <0:
        loss += (0-y)*out_of_view_k[1]
    elif y > img_width:
        loss += (y-img_height)*out_of_view_k[1]
    return loss

# # 定义自定义的目标函数（例如：最小化一个简单的二次函数）
# def loss_function_pixel(params):
#     return -gauss_left.val(x= params[0], y=params[1])

def d_normal_dist_loss(d):
    global phase
    # 计算正态分布的 PDF(概率密度)
    pdf_value = d_normal_distributions[phase].log_prob(d).exp()
    return -pdf_value

def loss_function(lap_X):
    global inst_0_p_dict, T_0_rcm, phase, pixel_adjust_dict
    T_rcm_cam = T_rcm_shaft_calculate(lap_X) @ T_shaft_cam
    # return torch.sum(T_rcm_cam )
    # return torch.sum(rotation_z_T(lap_X[1]) @ rotation_x_T(lap_X[0]) @  rotation_y_T(lap_X[2]) @ ( T_rcm_shaft_0))
    T_0_cam = T_0_rcm @ T_rcm_cam
    # print(f"T_0_cam:\n{T_0_cam}")
    # return torch.sum(T_0_cam)
    pixel_loss_list = []
    for inst in inst_in_phase[phase]:
        pixel_p = pixel_position(T_0_cam, inst_0_p_dict[inst])
        pixel_p = pixel_p + pixel_adjust_dict[phase]
        if not inst in  inst_in_phase[phase]: continue
        try:
            pixel_loss_list.append(pixel_loss(pixel_p, gauss_2d_dist_dict[phase][inst]))
        except:
            pass
    # pixel_p1 = pixel_position(T_0_cam, inst_0_p1)
    # pixel_p2 = pixel_position(T_0_cam, inst_0_p2)
    # print(f"inst left:{ inst_0_p1}, right:{inst_0_p2}, \ncam:\n{T_0_cam}")
    # print(f"pixel: {pixel_p1.data},  {pixel_p2.data}")
    if LAP_X_DOF == 4:
        loss_list = pixel_loss_list + [d_normal_dist_loss(lap_X[3])]
    else:
        loss_list = pixel_loss_list + [d_normal_dist_loss(lap_X[2])]
    loss = sum(loss_list)
    # loss = pixel_loss(pixel_p1, gauss_left) + pixel_loss(pixel_p2, gauss_right) + out_of_view_loss(pixel_p1) + out_of_view_loss(pixel_p2) + torch.abs(3*lap_X[2]) #+ torch.pow(0.1,  10* lap_X[3])
    # loss = pixel_loss(pixel_p1, gauss_left) + out_of_view_loss(pixel_p1)
    # loss = pixel_loss(pixel_p2, gauss_right, 10) + out_of_view_loss(pixel_p2)
    # loss = pixel_loss(pixel_p1, gauss_left) + pixel_loss(pixel_p2, gauss_right)
    # print(f"loss: {loss_list}")
    return loss



# 定义优化器
# optimizer = optim.AdamW([lap_X],    lr=0.001,             # 学习率
#                                 betas=(0.3, 0.1),  # 一阶和二阶矩的衰减系数
#                                 eps=1e-8,            # 防止除零的小常数
#                                 weight_decay=0.001,   # 权重衰减
#                                 amsgrad=False        # 是否使用AMSGrad算法
#                                 )

optimizer = optim.Adam([lap_X], 
                       lr=1e-3,          # 学习率
                       betas=(0.9, 0.999), # 一阶和二阶矩的衰减率
                       weight_decay=0.001) # 权重衰减

# optimizer = optim.SGD([lap_X], 
#                       lr=1e-3,           # 学习率
#                       momentum=0.3,      # 动量
#                       dampening=0,       # 动量衰减
#                       weight_decay=0.001,# 权重衰减
#                       nesterov=True)     # 使用Nesterov动量



if draw_on:
    plt.ion()  # 开启交互模式
    # 创建图像窗口
    plt.figure(figsize=(10, 6))

    # 计算并绘制整个区域的高斯值
    x_range = np.linspace(-img_width, 2*img_width, 100)
    y_range = np.linspace(-img_height, 2*img_height, 60)
    X, Y = np.meshgrid(x_range, y_range)

    # 计算每个点的高斯值
    draw_Z = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            gauss_val = 0
            for inst in inst_in_phase[phase]:
                gauss_val += gauss_2d_dist_dict[phase][inst].val(x= torch.tensor(X[i, j]), y= torch.tensor(Y[i, j]))
            draw_Z[i, j] = gauss_val

    # 绘制热力图
    plt.imshow(draw_Z, extent=[0, img_width, 0, img_height], origin='lower', cmap='viridis', aspect='auto')
    plt.colorbar(label='Gaussian Value')
    plt.draw()
    plt.show()
    # plt.pause(5)


def optimize_epoches(epoch_num):
    global lap_X, inst_0_p_dict, draw_Z, draw_and_save_id
    data_list = []
    draw_this_optimize = True
    # if draw_and_save_id == 0:
    #     draw_this_optimize = True
    # else:
    #     draw_this_optimize = False

    if draw_on and draw_this_optimize:
            plt.clf()
            plt.imshow(draw_Z, extent=[0, img_width, 0, img_height], origin='lower', cmap='viridis', aspect='auto')
            colors = [(217/255,120/255,45/255), 'red']


    start_time = time.perf_counter()
    pixel_p_list = []
    # 优化过程
    for epoch in range(epoch_num):
        data = []
        loop_start_time = time.perf_counter()
         # 清零梯度
        optimizer.zero_grad()

        # 计算损失
        loss = loss_function(lap_X)

        # 反向传播计算梯度
        loss.backward()
        # print(f"Loss: {loss.item()}, ")
        # 打印 lap_X 的梯度
        # print(f"epoch: {epoch}, \tlap_X:{[f'{x:.2f}' for x in lap_X.tolist()]}, lap_X.grad:{lap_X.grad.data}" )
        if epoch == 0:
            
            #  epoch+1  loss            lap_X
            data=[0,    loss.item()] +  lap_X.tolist() + lap_X.grad.data.tolist() 
            data_list.append(data)
        # 更新参数
        optimizer.step()
        
        T_0_cam = T_0_rcm @ T_rcm_shaft_calculate(lap_X) @ T_shaft_cam
        # pixel_p1 = pixel_position(T_0_cam, inst_0_p1)
        # pixel_p2 = pixel_position(T_0_cam, inst_0_p2)

        #  epoch+1      loss            lap_X
        data=[epoch+1,  loss.item()] +  lap_X.tolist() + lap_X.grad.data.tolist() 
        data_list.append(data)

        if draw_on and draw_this_optimize:
            pixel_data = []
            tempm_label = ["Dissector", "Needle Holder"]
            # 设置支持中文字体
            # plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC"] #字体
            # plt.rcParams["axes.unicode_minus"] = False  # 解决负号显示问题
            fontsize_all = 18
            color_all = 'black'
            if epoch == 0:
                for i in range(len(inst_in_phase[phase])):
                    inst = inst_in_phase[phase][i]
                    pixel_p = pixel_position(T_0_cam, inst_0_p_dict[inst]) + pixel_adjust_dict[phase]
                    # plt.scatter(pixel_p[0].item(), pixel_p[1].item(), color=colors[i], s=5, label=f'{inst}')
                    plt.scatter(pixel_p[0].item(), pixel_p[1].item(), color=colors[i], s=5, label=f'{tempm_label[i]}')
                    plt.text(pixel_p[0].item(), pixel_p[1].item(),f'{epoch} ', color = color_all, fontsize=fontsize_all-4)
            elif (epoch+1) %10 == 0:
                for i in range(len(inst_in_phase[phase])):
                    inst = inst_in_phase[phase][i]
                    pixel_p = pixel_position(T_0_cam, inst_0_p_dict[inst]) + pixel_adjust_dict[phase]
                    plt.scatter(pixel_p[0].item(), pixel_p[1].item(), color=colors[i], s=5)
                    if epoch == 99:
                        plt.text(pixel_p[0].item(), pixel_p[1].item(),f'{epoch+1} ', color=color_all, fontsize=fontsize_all-4)
            for i in range(len(inst_in_phase[phase])):
                inst = inst_in_phase[phase][i]
                pixel_p = pixel_position(T_0_cam, inst_0_p_dict[inst]) + pixel_adjust_dict[phase]
                pixel_data.append(np.array([pixel_p[0].item(), pixel_p[1].item()]))
            pixel_p_list.append(pixel_data)

                # 绘制当前优化的坐标
                # plt.scatter(pixel_p1[0].item(), pixel_p1[1].item(), color=(217/255,120/255,45/255), s=5, label=f'Epoch {epoch + 1}')
                # plt.text(pixel_p1[0].item(), pixel_p1[1].item(),f'{epoch} ', color='white', fontsize=8)
                # plt.scatter(pixel_p2[0].item(), pixel_p2[1].item(), color='red', s=5, label=f'Epoch {epoch + 1}')
                # plt.text(pixel_p2[0].item(), pixel_p2[1].item(),f'{epoch} ', color='white', fontsize=8)

        if epoch % 10 == 0:
            # lap_X_record_file.write(f"{time.perf_counter()},{phase},{epoch},{np.array2string(lap_X.detach().numpy(), separator=' ')}\n")
            pass

        # # 打印当前损失和优化参数
        # if epoch % 1 == 0:
        #     print(f"{epoch}, Loss:{loss.item():.4f}, lap_X:{[f'{x:.2f}' for x in lap_X.tolist()]}, Grad: {lap_X.grad},\
        #         pixel:{[f'{s:.1f}' for s in pixel_p1.detach().numpy()[0:2]]},{[f'{s:.1f}' for s in pixel_p2.detach().numpy()[0:2]]},\
        #         one loop: {float(time.perf_counter()-loop_start_time):.3f} s")
            
    duration = time.perf_counter() - start_time
    # print(f"loop  lap_X:{[f'{x:.3f}' for x in lap_X.tolist()]},\tduration: {duration:.3f}")

    if draw_on and draw_this_optimize:
        # 显示最终图像
        plt.xlabel('X', fontsize = fontsize_all)
        plt.ylabel('Y', fontsize = fontsize_all)
        plt.tick_params(axis='both', labelsize = fontsize_all)
        # plt.gca().invert_yaxis()
        # plt.title('Filed of View Optimization')
        plt.legend(fontsize = fontsize_all)
        plt.gca().invert_yaxis()
        # plt.show()
        if show_on:
            plt.draw()
            plt.pause(0.01)
        # plt.savefig(f"{optimal_offline_folder}/0.png")
        plt.savefig(f"{optimal_offline_folder}/{draw_and_save_id}.png")
    np.savetxt(f"{optimal_offline_folder}/optimal_data.txt", data_list, delimiter=',')
    draw_and_save_id += 1
    duration = time.perf_counter() - start_time
    print(f"whole duration: {duration:.3f}")
    if not pixel_p_list[0] and not pixel_p_list[-1]:
        delta_pixel_max = 0
    else:
        delta_pixel_1 = np.linalg.norm( pixel_p_list[0][0]-pixel_p_list[-1][0])
        if len(pixel_p_list[0]) <= 1:
            delta_pixel_2 = 0
        else:
            delta_pixel_2 = np.linalg.norm( pixel_p_list[0][1]-pixel_p_list[-1][1])
        delta_pixel_max = max(delta_pixel_1, delta_pixel_2)
    return delta_pixel_max, delta_pixel_1, delta_pixel_2

#==========================  本文件中添加的函数  ========================================================================================
def load_rigid_end_txt_to_dicts(file_path):
    """
    将原本一帧多行的txt转换为dicts_list

    """
    rigid_end_np = np.loadtxt(file_path, dtype=str, delimiter='\t')

    timestamp = None
    list = []
    dict = {}
    for i in range(rigid_end_np.shape[0]):
        if float(rigid_end_np[i][0]) != timestamp:
            if dict != {}:
                list.append(dict.copy())
                dict = {}
            timestamp = float(rigid_end_np[i][0])
            position = np.array(rigid_end_np[i][3].split(','), dtype=float)
            dict.update({"timestamp": timestamp,
                         rigid_end_np[i][2]:position})
        else:
            position = np.array(rigid_end_np[i][3].split(','), dtype=float)
            dict.update({rigid_end_np[i][2]:position})
    list.append(dict.copy())
    # print(f"rigid_end_dict[100]:\n{list[100]}")
    # print(f"rigid_end_dict[101]:\n{list[101]}")
    return list
            


def find_closest_element_in_dicts_list(dicts_list, target_timestamp, key = "timestamp"):
    """
    在存储有很多字典的列表中, 找到字典中 key 对应值与给定的target最接近的一个，返回这个字典和它在列表中的序号
    Args:
        dicts_list : 字典列表
        target_timestamp : 目标值

    Returns:
        字典序号, 字典
    """
    # 找到最接近的元素
    closest_element = min(dicts_list, key=lambda x: abs(x[key] - target_timestamp))
    # 获取索引
    closest_index = dicts_list.index(closest_element)
    return closest_index, closest_element


def find_closest_row_in_matrix(np_matrix, target_timestamp):
    """
    和 find_closest_element_in_dicts_list 类似，只是从矩阵每行中找
    """
    # 将第一列（时间戳）转换为 float 类型
    timestamps = np_matrix[:, 0].astype(float)
    # 计算时间差的绝对值，找到最小值的索引
    closest_index = np.abs(timestamps - target_timestamp).argmin()
    # 获取最接近的行
    closest_row = np_matrix[closest_index,:].squeeze()

    return closest_index, closest_row

def load_data():
    global lap_pose_pub_cmd, rigid_end, grad_opt_lap_X

    with open(lap_pose_pub_cmd_file,"r", encoding="utf-8") as json_file:
        rawtext = json_file.read()
        fixed_json = "[\n" + rawtext.replace("}{", "},\n{") + "\n]"
        lap_pose_pub_cmd = json.loads(fixed_json)
    print(f"lap_pose_pub_cmd length: {len(lap_pose_pub_cmd)}")

    rigid_end = load_rigid_end_txt_to_dicts(rigid_end_file)
    print(f"rigid_end length: {len(rigid_end)}")

    grad_opt_lap_X = np.loadtxt(grad_opt_lap_X_file, dtype=str, delimiter=',')
    print(f"grad_opt_lap_X shape: {grad_opt_lap_X.shape}")

    
def variables_update(id_lap_pose_pub_cmd):
    global lap_pose_pub_cmd, rigid_end, grad_opt_lap_X
    global lap_X_now
    global base0_rigid_end_dict, inst_0_p_dict
    global phase, draw_Z, inst_changing

    timestamp = lap_pose_pub_cmd[id_lap_pose_pub_cmd]["timestamp"]

    lap_X_now = torch.tensor(lap_pose_pub_cmd[id_lap_pose_pub_cmd]["lap_X_now"], dtype=torch.float32, requires_grad= False)
    
    id_grad_opt_lap_X, grad_opt_lap_X_row  = find_closest_row_in_matrix(grad_opt_lap_X, timestamp)
    phase = grad_opt_lap_X_row[1]

    id_rigid_end, rigid_end_data = find_closest_element_in_dicts_list(rigid_end, timestamp)
    for inst in inst_in_phase[phase]:
        if inst in rigid_end_data:
            inst_0_p_dict[inst] = torch.tensor(T_0_qualisys @ np.append((rigid_end_data[inst]), 1), dtype= torch.float32) #！！！手动调节了坐标
            # inst_0_p_dict[inst] = torch.tensor(T_0_qualisys @ np.append((rigid_end_data[inst]+np.array([-0.05, 0.15, 0])), 1), dtype= torch.float32) #！！！手动调节了坐标
    
    draw_Z = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            gauss_val = 0
            for inst in inst_in_phase[phase]:
                gauss_val += gauss_2d_dist_dict[phase][inst].val(x= torch.tensor(X[i, j]), y= torch.tensor(Y[i, j]))
            draw_Z[i, j] = gauss_val

    print(f"lap_X_now:{lap_X_now}")
    print(f"inst_0_p_dict:{inst_0_p_dict}")
    print(f"phase:{phase}")
    
    



    
if __name__ == "__main__":
    data_index = 40000 #选取 lap_pose_pub_cmd 中的第几帧数据进行分析
    load_data()
    # variables_update(data_index)
  

    time.sleep(1)
    delta_pixel_list = []
    for i in range(1000, 70000, 100):
    # for i in [62405]:
    # for i in [12200]:
        print(f"{i} ---------------------------")
        variables_update(i)
        skip_while = False
        for inst in inst_in_phase[phase]:
            if inst_0_p_dict[inst] is None:
                print(f"inst_0_p_dict {inst} : None")
                skip_while = True
                continue
        if skip_while:
            exit()
        
        if lap_X_now is not None:
            if LAP_X_DOF == 4:
                lap_X.data = lap_X_now
            else:
                lap_X.data[0] = lap_X_now[0]
                lap_X.data[1] = lap_X_now[1]
                lap_X.data[2] = lap_X_now[3]

        # print(f"lap_X:{lap_X}")
        delta_pixel_max, delta_pixel_1, delta_pixel_2 = optimize_epoches(100)
        print(f"delta pixel: max:{delta_pixel_max}  1:{delta_pixel_1}  2:{delta_pixel_2}")
        if delta_pixel_max > 180:
            delta_pixel_list.append([1, i, delta_pixel_max, delta_pixel_1, delta_pixel_2])
        else:
            delta_pixel_list.append([0, i, delta_pixel_max, delta_pixel_1, delta_pixel_2])

        # print(f"lap_X:{lap_X}")
        if not torch.isnan(lap_X).any():
            lap_X_val = lap_X.detach().numpy()

            if lap_X_val[0] > alpha_max: lap_X_val[0] = alpha_max
            elif lap_X_val[0] < alpha_min: lap_X_val[0] = alpha_min

            if lap_X_val[1] > beta_max: lap_X_val[1] = beta_max
            elif lap_X_val[1] < beta_min: lap_X_val[1] = beta_min

            

            if LAP_X_DOF == 4:
                if lap_X_val[3] > d_max: lap_X_val[3] = d_max
                elif lap_X_val[3] < d_min: lap_X_val[3] = d_min

            else:
                if lap_X_val[2] > d_max: lap_X_val[2] = d_max
                elif lap_X_val[2] < d_min: lap_X_val[2] = d_min


            if inst_changing:
                time.sleep(3)
                inst_changing = False
    np.savetxt(f"{optimal_offline_folder}/delta_pixel.txt", delta_pixel_list, delimiter=',')

  