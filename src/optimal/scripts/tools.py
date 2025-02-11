import numpy as np
import torch
from torch import tensor
import torch.optim as optim
import time
import matplotlib.pyplot as plt


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

def rotation_x_R_np(theta):
    # return torch.tensor([
    #     [1,     0,                  0                   ],
    #     [0,     torch.cos(theta),   -torch.sin(theta)   ],
    #     [0,     torch.sin(theta),   torch.cos(theta)    ]
    # ], requires_grad= True )
    R_array = np.zeros((3,3))
    R_array[0, 0] = 1
    R_array[1, 1] = np.cos(theta)
    R_array[1, 2] = -np.sin(theta)
    R_array[2, 1] = np.sin(theta)
    R_array[2, 2] = np.cos(theta)
    return R_array

def rotation_y_R_np(theta):
    # np.tensor([
    #     [np.cos(theta),      0,      np.sin(theta)],
    #     [0,                     1,      0               ],
    #     [-np.sin(theta),     0,      np.cos(theta)]
    # ], requires_grad= True )
    R_array = np.zeros((3,3))
    R_array[0, 0] = np.cos(theta)
    R_array[0, 2] = np.sin(theta)
    R_array[1, 1] = 1
    R_array[2, 0] = -np.sin(theta)
    R_array[2, 2] = np.cos(theta)
    return R_array

def rotation_z_R_np(theta):
    # np.tensor([
    #     [np.cos(theta),  -np.sin(theta),      0],
    #     [np.sin(theta),  np.cos(theta),       0],
    #     [0,                 0,                      1]
    # ], requires_grad= True )
    R_array = np.zeros((3,3))
    R_array[0, 0] = np.cos(theta)
    R_array[0, 1] = -np.sin(theta)
    R_array[1, 0] = np.sin(theta)
    R_array[1, 1] = np.cos(theta)
    R_array[2, 2] = 1
    return R_array

def rotation_x_T_np(theta):
    # return np.tensor([
    #     [1,     0,                  0,                  0],
    #     [0,     np.cos(theta),   -np.sin(theta),  0],
    #     [0,     np.sin(theta),   np.cos(theta),   0],
    #     [0,     0,                  0,                  1]
    # ], requires_grad= True )
    T_array = np.zeros((4,4))
    T_array[0, 0] = 1
    T_array[1, 1] = np.cos(theta)
    T_array[1, 2] = -np.sin(theta)
    T_array[2, 1] = np.sin(theta)
    T_array[2, 2] = np.cos(theta)
    T_array[3, 3] = 1
    return T_array

def rotation_y_T_np(theta):
    # np.tensor([
    #     [np.cos(theta),      0,      np.sin(theta),   0],
    #     [0,                     1,      0,                  0],
    #     [-np.sin(theta),     0,      np.cos(theta),   0],
    #     [0,                     0,      0,                  1]
    # ], requires_grad= True )
    T_array = np.zeros((4,4))
    T_array[0, 0] = np.cos(theta)
    T_array[0, 2] = np.sin(theta)
    T_array[1, 1] = 1
    T_array[2, 0] = -np.sin(theta)
    T_array[2, 2] = np.cos(theta)
    T_array[3, 3] = 1
    return T_array

def rotation_z_T_np(theta):
    # np.tensor([
    #     [np.cos(theta),  -np.sin(theta),      0,  0],
    #     [np.sin(theta),  np.cos(theta),       0,  0],
    #     [0,                 0,                      1,  0],
    #     [0,                 0,                      0,  1]
    # ], requires_grad= True )
    T_array = np.zeros((4,4))
    T_array[0, 0] = np.cos(theta)
    T_array[0, 1] = -np.sin(theta)
    T_array[1, 0] = np.sin(theta)
    T_array[1, 1] = np.cos(theta)
    T_array[2, 2] = 1
    T_array[3, 3] = 1
    return T_array

def T_inv_np(T):
    """
    齐次变换矩阵求逆
    """
    R_matrix = T[:3, :3]
    p = T[:3, 3]

    # 计算 R 的转置
    R_inv = R_matrix.T
    p_inv = -R_inv @ p
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = p_inv
    return T_inv
