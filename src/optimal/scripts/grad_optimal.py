import numpy as np
import torch
from torch import tensor
import torch.optim as optim
import time
import matplotlib.pyplot as plt
# from autograd import grad
# import autograd.numpy as gradnp

'''
f_0 是一个尺度参数，影响整个函数的幅度。
f_1 是旋转角度参数，表示二维高斯的旋转。
f_2 和 f_3 是两个标准差参数，控制高斯分布的形状。
f_4 和 f_5 是二维高斯分布的中心坐标。
f_6 是一个常数偏移项。
'''
'''                  幅度，       旋转角度，   x标准差,        y标准差,       x中心,       y中心,         常数偏移  '''
fun_left_0 = [5.410334, 137.06098690, 377.40041913, 431.40073990, 400.18590305, 580.16764162, 0.00402319]
fun_right_0 = [7.491880, 96.42623581, 99.32096722, 254.80939073, 1300.58609198, 580.94976255, 0.01276680] 
# fun_left_0 = [5.410334, 137.06098690, 1377.40041913, 1431.40073990, 400.18590305, 580.16764162, 0.00402319]
# fun_right_0 = [7.491880, 96.42623581, 1199.32096722, 1254.80939073, 1300.58609198, 580.94976255, 0.01276680] 

out_of_view_k = tensor([0.02, 0.02])

img_width = 1920 
img_height = 1080

draw_on = True

# 初始化优化参数（x）
lap_X = torch.tensor([-30.0/180.0*torch.pi, 0.0, 0.0, 0.15], requires_grad= True)  # 初始值为0，并启用梯度计算

inst_0_p1 = tensor([-0.02, 0.21, 0.03, 1]) #0坐标系下器械位置
inst_0_p2 = tensor([0.02, 0.21, 0.03, 1])

cam_K = tensor([
    [1.321383458370769176e+03,  0,                          9.680885724568087198e+02,   0],
    [0,                         1.323800731972558879e+03,   5.572575084185185688e+02,   0],
    [0,                         0,                          1,                          0]
])

T_0_rcm = tensor([
    [1, 0,  0,  0],
    [0, -1, 0,  0],
    [0, 0,  -1, 0.21],
    [0, 0,  0,  1]
])

class instrument():
    def __init__(self, name):
        self.name = name

def rotation_x_R(theta):
    # return torch.tensor([
    #     [1,     0,                  0                   ],
    #     [0,     torch.cos(theta),   -torch.sin(theta)   ],
    #     [0,     torch.sin(theta),   torch.cos(theta)    ]
    # ], requires_grad= True )
    R_tensor = torch.zeros(3, 3, device=theta.device)
    R_tensor[0, 0] = 1
    R_tensor[1, 1] = torch.cos(theta)
    R_tensor[1, 2] = -torch.sin(theta)
    R_tensor[2, 1] = torch.sin(theta)
    R_tensor[2, 2] = torch.cos(theta)
    return R_tensor

def rotation_y_R(theta):
    # torch.tensor([
    #     [torch.cos(theta),      0,      torch.sin(theta)],
    #     [0,                     1,      0               ],
    #     [-torch.sin(theta),     0,      torch.cos(theta)]
    # ], requires_grad= True )
    R_tensor = torch.zeros(3, 3, device=theta.device)
    R_tensor[0, 0] = torch.cos(theta)
    R_tensor[0, 2] = torch.sin(theta)
    R_tensor[1, 1] = 1
    R_tensor[2, 0] = -torch.sin(theta)
    R_tensor[2, 2] = torch.cos(theta)
    return R_tensor

def rotation_z_R(theta):
    # torch.tensor([
    #     [torch.cos(theta),  -torch.sin(theta),      0],
    #     [torch.sin(theta),  torch.cos(theta),       0],
    #     [0,                 0,                      1]
    # ], requires_grad= True )
    R_tensor = torch.zeros(3, 3, device=theta.device)
    R_tensor[0, 0] = torch.cos(theta)
    R_tensor[0, 1] = -torch.sin(theta)
    R_tensor[1, 0] = torch.sin(theta)
    R_tensor[1, 1] = torch.cos(theta)
    R_tensor[2, 2] = 1
    return R_tensor

def rotation_x_T(theta):
    # return torch.tensor([
    #     [1,     0,                  0,                  0],
    #     [0,     torch.cos(theta),   -torch.sin(theta),  0],
    #     [0,     torch.sin(theta),   torch.cos(theta),   0],
    #     [0,     0,                  0,                  1]
    # ], requires_grad= True )
    T_tensor = torch.zeros(4, 4, device=theta.device)
    T_tensor[0, 0] = 1
    T_tensor[1, 1] = torch.cos(theta)
    T_tensor[1, 2] = -torch.sin(theta)
    T_tensor[2, 1] = torch.sin(theta)
    T_tensor[2, 2] = torch.cos(theta)
    T_tensor[3, 3] = 1
    return T_tensor

def rotation_y_T(theta):
    # torch.tensor([
    #     [torch.cos(theta),      0,      torch.sin(theta),   0],
    #     [0,                     1,      0,                  0],
    #     [-torch.sin(theta),     0,      torch.cos(theta),   0],
    #     [0,                     0,      0,                  1]
    # ], requires_grad= True )
    T_tensor = torch.zeros(4, 4, device=theta.device)
    T_tensor[0, 0] = torch.cos(theta)
    T_tensor[0, 2] = torch.sin(theta)
    T_tensor[1, 1] = 1
    T_tensor[2, 0] = -torch.sin(theta)
    T_tensor[2, 2] = torch.cos(theta)
    T_tensor[3, 3] = 1
    return T_tensor

def rotation_z_T(theta):
    # torch.tensor([
    #     [torch.cos(theta),  -torch.sin(theta),      0,  0],
    #     [torch.sin(theta),  torch.cos(theta),       0,  0],
    #     [0,                 0,                      1,  0],
    #     [0,                 0,                      0,  1]
    # ], requires_grad= True )
    T_tensor = torch.zeros(4, 4, device=theta.device)
    T_tensor[0, 0] = torch.cos(theta)
    T_tensor[0, 1] = -torch.sin(theta)
    T_tensor[1, 0] = torch.sin(theta)
    T_tensor[1, 1] = torch.cos(theta)
    T_tensor[2, 2] = 1
    T_tensor[3, 3] = 1
    return T_tensor

def T_inv(T):
    """
    齐次变换矩阵求逆
    """
    R_tensor = T[:3, :3]
    p = T[:3, 3]

    # 计算 R 的转置
    R_inv = R_tensor.T
    p_inv = -R_inv @ p
    
    T_inv = torch.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = p_inv
    return T_inv


T_shaft_cam = rotation_x_T(tensor(-torch.pi/6))
T_rcm_shaft_0 = tensor([
    [1, 0,  0,  0],
    [0, 0,  -1, 0],
    [0, 1,  0,  0],
    [0, 0,  0,  1]
], dtype= torch.float32)

def T_rcm_cam_calculate(lap_X):
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

def loss_function(lap_X):
    global inst_0_p1, inst_0_p2, T_0_rcm
    T_rcm_cam = T_rcm_cam_calculate(lap_X)
    # return torch.sum(T_rcm_cam )
    # return torch.sum(rotation_z_T(lap_X[1]) @ rotation_x_T(lap_X[0]) @  rotation_y_T(lap_X[2]) @ ( T_rcm_shaft_0))
    T_0_cam = T_0_rcm @ T_rcm_cam
    # return torch.sum(T_0_cam)
    pixel_p1 = pixel_position(T_0_cam, inst_0_p1)
    pixel_p2 = pixel_position(T_0_cam, inst_0_p2)
    loss = pixel_loss(pixel_p1, gauss_left) + pixel_loss(pixel_p2, gauss_right) + out_of_view_loss(pixel_p1) + out_of_view_loss(pixel_p2)
    # loss = pixel_loss(pixel_p1, gauss_left) + pixel_loss(pixel_p2, gauss_right)
    return loss



# 定义优化器
optimizer = optim.AdamW([lap_X],    lr=0.01,             # 学习率
                                betas=(0.1, 0.1),  # 一阶和二阶矩的衰减系数
                                eps=1e-8,            # 防止除零的小常数
                                weight_decay=0.005,   # 权重衰减
                                amsgrad=False        # 是否使用AMSGrad算法
                                )


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
    global lap_X
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
        # print("lap_X.grad:", lap_X.grad)
        # 更新参数
        optimizer.step()
        
        T_0_cam = T_0_rcm @ T_rcm_cam_calculate(lap_X)
        pixel_p1 = pixel_position(T_0_cam, inst_0_p1)
        pixel_p2 = pixel_position(T_0_cam, inst_0_p2)

        if draw_on:
        # 绘制当前优化的坐标
            plt.scatter(pixel_p1[0].item(), pixel_p1[1].item(), color=(217/255,120/255,45/255), s=5, label=f'Epoch {epoch + 1}')
            plt.text(pixel_p1[0].item(), pixel_p1[1].item(),f'{epoch} ', color='white', fontsize=8)
            plt.scatter(pixel_p2[0].item(), pixel_p2[1].item(), color='red', s=5, label=f'Epoch {epoch + 1}')
            plt.text(pixel_p2[0].item(), pixel_p2[1].item(),f'{epoch} ', color='white', fontsize=8)


        # 打印当前损失和优化参数
        if epoch % 1 == 0:
            print(f"{epoch}, Loss:{loss.item():.4f}, lap_X:{[f'{x:.2f}' for x in lap_X.tolist()]}, Grad: {lap_X.grad},\
                pixel:{[f'{s:.1f}' for s in pixel_p1.detach().numpy()[0:2]]},{[f'{s:.1f}' for s in pixel_p2.detach().numpy()[0:2]]},\
                one loop: {float(time.perf_counter()-loop_start_time):.3f} s")
            
    duration = time.perf_counter() - start_time
    print(f"durationg: {duration:.3f}")

    if draw_on:
        # 显示最终图像
        plt.xlabel('X coordinate')
        plt.ylabel('Y coordinate')
        plt.gca().invert_yaxis()
        plt.title('2D Gaussian Distribution with Optimized Coordinates')
        plt.legend()
        plt.show()

if __name__ == "__main__":
    print(f"lap_X:{lap_X}")
    optimize_epoches(20)
    print(f"lap_X:{lap_X}")