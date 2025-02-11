"""
    根据实时腹腔镜位姿和手术器械末端位置
    优化腹腔镜位姿
    并发布优化结果

    订阅：
        /state/lap_X    实时腹腔镜4 dof 位姿 
        /base0_rigid_end_pub    base0坐标系下手术器械末端位置
    
    发布：
        /cmd/lap_X  优化后的腹腔镜位姿
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


sys.path.append(f"{os.path.dirname(__file__)}")
from lap_set_pk import lap_set
from tools import *
#============================================================================================
LAP_X_DOF = 3 #alpha、beta、gamma、d 4自由度，如果设为3，则不优化 gamma = 0
draw_on = False

'''
f_0 是一个尺度参数，影响整个函数的幅度。
f_1 是旋转角度参数，表示二维高斯的旋转。
f_2 和 f_3 是两个标准差参数，控制高斯分布的形状。
f_4 和 f_5 是二维高斯分布的中心坐标。
f_6 是一个常数偏移项。
'''
'''                  幅度，       旋转角度，   x标准差,        y标准差,       x中心,       y中心,         常数偏移  '''
fun_left_0 = [5.410334, 137.06098690, 377.40041913, 431.40073990, 400.18590305, 580.16764162, 0.00402319]
# fun_right_0 = [7.491880, 96.42623581, 99.32096722, 254.80939073, 1300.58609198, 580.94976255, 0.01276680] 
fun_right_0 = [7.491880, 96.42623581, 299.32096722, 354.80939073, 1300.58609198, 580.94976255, 0.01276680] 
# fun_left_0 = [5.410334, 137.06098690, 1377.40041913, 1431.40073990, 400.18590305, 580.16764162, 0.00402319]
# fun_right_0 = [7.491880, 96.42623581, 1199.32096722, 1254.80939073, 1300.58609198, 580.94976255, 0.01276680] 

out_of_view_k = tensor([0.02, 0.02])

img_width = lap_set.video_width
img_height = lap_set.video_height

base0_rigid_end_dict = {}

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
phases_list = ["gripping", "needling", "tightening", "knotting", "cutting", "moving", "grabbing", "placing"]
d_mu = {
    "gripping": torch.tensor(0.15950481412772527),
    "needling": torch.tensor(0.1859067586628832),
    "tightening": torch.tensor(0.13961117283683613),
    "knotting": torch.tensor(0.16773371255903682),
    "cutting": torch.tensor(0.15678583345909003),
    "moving": torch.tensor(0.11960882171313425),
    "grabbing": torch.tensor(0.17587754305798572),
    "placing": torch.tensor(0.15800867650553185)
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
for phase in phases_list:
    d_normal_distributions[phase] = torch.distributions.Normal(d_mu[phase], d_sigma[phase])
phase = "gripping"

if LAP_X_DOF == 4:
    # 初始化优化参数（x）,用于优化，和发布控制指令，作为desire
    lap_X = torch.tensor([-20.0/180.0*torch.pi, 0.0, 0.0, 0.15], requires_grad= True)  # 初始值为0，并启用梯度计算
if LAP_X_DOF == 3:
    lap_X = torch.tensor([-20.0/180.0*torch.pi, 0.0, 0.15], requires_grad= True)  # 初始值为0，并启用梯度计算
lap_X_now = None #接收到的现在机器人的 lap_X 实际值



# inst_0_p1 = tensor([-0.02, 0.21, 0.03, 1]) #0坐标系下器械位置
# inst_0_p2 = tensor([0.02, 0.21, 0.03, 1])
inst_0_p1 = None #0坐标系下器械位置
inst_0_p2 = None


Camera_Calibration_folder = f'{lap_set.data_folder}/Camera_Calibration'
T_rob_camera = np.loadtxt(f"{Camera_Calibration_folder}/camera_tool.csv")
T_rob_camera = torch.tensor(T_rob_camera)


cam_K = tensor([
    [1.321383458370769176e+03,  0,                          9.680885724568087198e+02,   0],
    [0,                         1.323800731972558879e+03,   5.572575084185185688e+02,   0],
    [0,                         0,                          1,                          0]
])

T_0_rcm = tensor(lap_set.T_0_rcm, dtype= torch.float32)

class instrument():
    def __init__(self, name):
        self.name = name


T_shaft_cam = rotation_x_T(tensor(-torch.pi/6))
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
    alpha = lap_X[0]
    beta = lap_X[1]
    gamma = lap_X[2]
    d = lap_X[3]
    add_d = torch.zeros(4, 4)
    add_d[1, 3] =-d
    T_rcm_cam = rotation_z_T(beta) @ rotation_x_T(alpha) @  rotation_y_T(gamma) @ (add_d + T_rcm_shaft_0)
    return T_rcm_cam

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

        val = (
            self.func_args[6]
            + self.func_args[0] * torch.exp(
                -(((x - self.func_args[4]) * torch.cos(self.func_args[1] * torch.pi / 180) + (y - self.func_args[5]) * torch.sin(self.func_args[1] * torch.pi / 180)) / self.func_args[2]) ** 2
                - ((-(x - self.func_args[4]) * torch.sin(self.func_args[1] * torch.pi / 180) + (y - self.func_args[5]) * torch.cos(self.func_args[1] * torch.pi / 180)) / self.func_args[3]) ** 2
            )
        )
        return val


def pixel_position(T_0_cam, inst_0_p):
    """
    计算器械末端投影的二维坐标

    Args:
        T_0_cam (4*4): 0 坐标系下 镜头坐标系(已旋转30°)
        inst_0_p (4): 0 坐标系下 器械末端齐次坐标 

    Returns:
        pixel_p(3): 像素齐次坐标[u, v, 1]
    """
    T_cam_0 = T_inv(T_0_cam)
    inst_cam_p = T_cam_0 @ inst_0_p
    z = inst_cam_p[2]
    pixel_p = cam_K @ inst_cam_p/z
    return pixel_p



gauss_left = Gaussian_Distribution_2D_tensor(fun_left_0)
gauss_right = Gaussian_Distribution_2D_tensor(fun_right_0)


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

# 定义自定义的目标函数（例如：最小化一个简单的二次函数）
def loss_function_pixel(params):
    return -gauss_left.val(x= params[0], y=params[1])

def d_normal_dist_loss(d):
    global phase
    # 计算正态分布的 PDF(概率密度)
    pdf_value = d_normal_distributions[phase].log_prob(d).exp()
    return -pdf_value

def loss_function(lap_X):
    global inst_0_p1, inst_0_p2, T_0_rcm
    T_rcm_cam = T_rcm_shaft_calculate(lap_X) @ T_shaft_cam
    # return torch.sum(T_rcm_cam )
    # return torch.sum(rotation_z_T(lap_X[1]) @ rotation_x_T(lap_X[0]) @  rotation_y_T(lap_X[2]) @ ( T_rcm_shaft_0))
    T_0_cam = T_0_rcm @ T_rcm_cam
    # return torch.sum(T_0_cam)
    pixel_p1 = pixel_position(T_0_cam, inst_0_p1)
    pixel_p2 = pixel_position(T_0_cam, inst_0_p2)
    loss_list = [pixel_loss(pixel_p1, gauss_left), pixel_loss(pixel_p2, gauss_right), out_of_view_loss(pixel_p1), out_of_view_loss(pixel_p2), d_normal_dist_loss(lap_X[3])]
    loss = sum(loss_list)
    # loss = pixel_loss(pixel_p1, gauss_left) + pixel_loss(pixel_p2, gauss_right) + out_of_view_loss(pixel_p1) + out_of_view_loss(pixel_p2) + torch.abs(3*lap_X[2]) #+ torch.pow(0.1,  10* lap_X[3])
    # loss = pixel_loss(pixel_p1, gauss_left) + out_of_view_loss(pixel_p1)
    # loss = pixel_loss(pixel_p2, gauss_right, 10) + out_of_view_loss(pixel_p2)
    # loss = pixel_loss(pixel_p1, gauss_left) + pixel_loss(pixel_p2, gauss_right)
    print(f"loss: {loss_list}")
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
    # 创建图像窗口
    plt.figure(figsize=(10, 6))

    # 计算并绘制整个区域的高斯值
    x_range = np.linspace(-img_width, 2*img_width, 100)
    y_range = np.linspace(-img_height, 2*img_height, 60)
    X, Y = np.meshgrid(x_range, y_range)

    # 计算每个点的高斯值
    Z = np.zeros_like(X)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            Z[i, j] = gauss_left.val(x= torch.tensor(X[i, j]), y= torch.tensor(Y[i, j])) + gauss_right.val(x= torch.tensor(X[i, j]), y= torch.tensor(Y[i, j]))

    # 绘制热力图
    plt.imshow(Z, extent=[0, img_width, 0, img_height], origin='lower', cmap='viridis', aspect='auto')
    plt.colorbar(label='Gaussian Value')


def optimize_epoches(epoch_num):
    global lap_X, inst_0_p1, inst_0_p2
    start_time = time.perf_counter()
    # 优化过程
    for epoch in range(epoch_num):
        loop_start_time = time.perf_counter()
        # 清零梯度
        optimizer.zero_grad()

        # 计算损失
        loss = loss_function(lap_X)

        # 反向传播计算梯度
        loss.backward()
        # print(f"Loss: {loss.item()}, ")
        # 打印 lap_X 的梯度
        print(f"lap_X:{[f'{x:.2f}' for x in lap_X.tolist()]}, lap_X.grad:{lap_X.grad}" )
        # 更新参数
        optimizer.step()
        
        T_0_cam = T_0_rcm @ T_rcm_shaft_calculate(lap_X) @ T_shaft_cam
        pixel_p1 = pixel_position(T_0_cam, inst_0_p1)
        pixel_p2 = pixel_position(T_0_cam, inst_0_p2)

        if draw_on:
            if epoch %10 == 0:
                # 绘制当前优化的坐标
                plt.scatter(pixel_p1[0].item(), pixel_p1[1].item(), color=(217/255,120/255,45/255), s=5, label=f'Epoch {epoch + 1}')
                plt.text(pixel_p1[0].item(), pixel_p1[1].item(),f'{epoch} ', color='white', fontsize=8)
                plt.scatter(pixel_p2[0].item(), pixel_p2[1].item(), color='red', s=5, label=f'Epoch {epoch + 1}')
                plt.text(pixel_p2[0].item(), pixel_p2[1].item(),f'{epoch} ', color='white', fontsize=8)


        # # 打印当前损失和优化参数
        # if epoch % 1 == 0:
        #     print(f"{epoch}, Loss:{loss.item():.4f}, lap_X:{[f'{x:.2f}' for x in lap_X.tolist()]}, Grad: {lap_X.grad},\
        #         pixel:{[f'{s:.1f}' for s in pixel_p1.detach().numpy()[0:2]]},{[f'{s:.1f}' for s in pixel_p2.detach().numpy()[0:2]]},\
        #         one loop: {float(time.perf_counter()-loop_start_time):.3f} s")
            
    duration = time.perf_counter() - start_time
    print(f"lap_X:{[f'{x:.2f}' for x in lap_X.tolist()]},\tdurationg: {duration:.3f}")

    if draw_on:
        # 显示最终图像
        plt.xlabel('X coordinate')
        plt.ylabel('Y coordinate')
        plt.gca().invert_yaxis()
        plt.title('2D Gaussian Distribution with Optimized Coordinates')
        plt.legend()
        plt.show()

def lap_X_callback(msg):
    global lap_X_now
    lap_X_now = torch.tensor(msg.data, dtype=torch.float32, requires_grad= False)
    # print("lap_X_callback")

def base0_rigid_end_callback(msg):
    global base0_rigid_end_dict, inst_0_p1, inst_0_p2
    base0_rigid_end_dict =  json.loads(msg.data)
    fenliqian_numpy = np.array(base0_rigid_end_dict['fenliqian'])
    chizhenqi_numpy = np.array(base0_rigid_end_dict['chizhenqi'])
    if fenliqian_numpy[0] is not np.nan:
        inst_0_p1 = torch.tensor(fenliqian_numpy, dtype= torch.float32)
    if chizhenqi_numpy[0] is not np.nan:
        inst_0_p2 = torch.tensor(chizhenqi_numpy,  dtype=torch.float32)
        



if __name__ == "__main__":
    rospy.init_node('grad_optimal_with_rob')

    rospy.Subscriber('/state/lap_X',Float32MultiArray, lap_X_callback)
    rospy.Subscriber('/base0_rigid_end_pub',String, base0_rigid_end_callback)
    lap_X_desired_pub = rospy.Publisher('/cmd/lap_X',Float32MultiArray,queue_size=1)
    lap_X_desired_msg = Float32MultiArray()

    while not rospy.is_shutdown():
        while inst_0_p1 is None or inst_0_p2 is None:
            pass
        
        if lap_X_now is not None:
            lap_X.data = lap_X_now
            if LAP_X_DOF == 3:
                lap_X[2] = 0.0

        # print(f"lap_X:{lap_X}")
        optimize_epoches(100)
        # print(f"lap_X:{lap_X}")
        if not torch.isnan(lap_X).any():
            lap_X_val = lap_X.detach().numpy()
            lap_X_desired_msg.data = lap_X_val.tolist()
            lap_X_desired_pub.publish(lap_X_desired_msg)