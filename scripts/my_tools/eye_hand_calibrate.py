'''
用于眼和机械臂分别固连在h世界的情况，注意眼不在手上。
机械臂末端固连观测目标，B
calibrate_calculate() 用于直接传入测得的机械臂末端变换矩阵和眼观测的目标位姿，解算手眼相对位姿

'''

import numpy as np
from spatialmath.base import *
import xml.etree.ElementTree as ET
import math
from scipy.linalg import logm


def shiu(A, B):
        '''
        解 AX=XB 方程组（对于ABX均为变换矩阵的情况）
        args:
            A: 4*4n,即将多个方程组的A横向拼接起来
            B: 4*4n
        return:
            X: 4*4
        '''
        # 计算 A 和 B 的维度
        if A.shape != B.shape:
            print(f'A、B 矩阵形状不同：{A.shape} != {B.shape}')
            return
        m, n = A.shape

        if n%4 != 0:
            print(f'矩阵列数({n}),不是4的整倍数')
            return
        n = n // 4


        # 初始化 AA 和 bb
        AA = np.zeros((9*(n-1), 2*n))
        bb = np.zeros((9*(n-1), 1))
        
        # 计算最佳旋转 R
        for i in range(n):
            print(f'i: {i}')
            A1 = logm(A[0:3, 4*i:4*i+3])
            B1 = logm(B[0:3, 4*i:4*i+3])
            a1 = np.array([A1[2, 1], A1[0, 2], A1[1, 0]]).squeeze()
            a1 = a1 / np.linalg.norm(a1)
            b1 = np.array([B1[2, 1], B1[0, 2], B1[1, 0]]).squeeze()
            b1 = b1 / np.linalg.norm(b1)
            v = np.cross(b1, a1)
            w = math.atan2(np.linalg.norm(v), np.dot(b1, a1))  
            v = v / np.linalg.norm(v)
            XP = np.eye(3) * np.cos(w) + np.sin(w) * skew(v) + (1 - np.cos(w)) * v.reshape(3,1) @ v.reshape(1,3)
            Ai, bi = shiu_matrix(a1, XP)
            if i == 0:
                AA[:, :2] = np.tile(-Ai, (n-1, 1))  
                bb[:, 0] = np.tile(-bi.reshape(-1,1), (n-1, 1)).squeeze()
            else:
                AA[9*(i-1):9*i, 2*i:2*i+2] = Ai
                bb[9*(i-1):9*i, 0] = bb[9*(i-1):9*i, 0] + bi.reshape(-1,1).squeeze()
        
        beta = np.linalg.lstsq(AA, bb, rcond=None)[0]
        theta = math.atan2(beta[2*n-1], beta[2*n-2])  # Convert beta components to scalar
        RA = np.eye(3) * np.cos(theta) + np.sin(theta) * skew(a1) +  a1.reshape(3,1) @ a1.reshape(1,3) * (1 - np.cos(theta)) 
        R = RA @ XP
        
        # 计算最佳平移 t
        C = np.zeros((3*n, 3))
        d = np.zeros((3*n, 1))
        I = np.eye(3)
        for i in range(n):
            C[3*i:3*i+3, :] = I - A[0:3, 4*i:4*i+3]
            d[3*i:3*i+3, :] = A[0:3, 4*i+3].reshape(-1,1) - R @ B[0:3, 4*i+3].reshape(-1,1)
        
        t = np.linalg.lstsq(C, d, rcond=None)[0]
        
        # 构建变换矩阵 X
        X = np.block([[R, t], [0, 0, 0, 1]])
        
        return X

def shiu_matrix(ka1, X):
    ka1 = ka1.squeeze()
    A = np.zeros((9, 2))
    b = np.zeros((9, 1))
    
    A[0, 0] = X[0, 0] - ka1[0] * np.dot(X[:, 0].squeeze(), ka1)
    A[1, 0] = X[0, 1] - ka1[0] * np.dot(X[:, 1].squeeze(), ka1)
    A[2, 0] = X[0, 2] - ka1[0] * np.dot(X[:, 2].squeeze(), ka1)
    A[3, 0] = X[1, 0] - ka1[1] * np.dot(X[:, 0].squeeze(), ka1)
    A[4, 0] = X[1, 1] - ka1[1] * np.dot(X[:, 1].squeeze(), ka1)
    A[5, 0] = X[1, 2] - ka1[1] * np.dot(X[:, 2].squeeze(), ka1)
    A[6, 0] = X[2, 0] - ka1[2] * np.dot(X[:, 0].squeeze(), ka1)
    A[7, 0] = X[2, 1] - ka1[2] * np.dot(X[:, 1].squeeze(), ka1)
    A[8, 0] = X[2, 2] - ka1[2] * np.dot(X[:, 2].squeeze(), ka1)
    
    n = np.cross(X[:, 0], ka1)
    o = np.cross(X[:, 1], ka1)
    a = np.cross(X[:, 2], ka1)
    
    A[0, 1] = -n[0]
    A[1, 1] = -o[0]
    A[2, 1] = -a[0]
    A[3, 1] = -n[1]
    A[4, 1] = -o[1]
    A[5, 1] = -a[1]
    A[6, 1] = -n[2]
    A[7, 1] = -o[2]
    A[8, 1] = -a[2]
    
    n = X[:, 0]
    o = X[:, 1]
    a = X[:, 2]
    
    b[0] = -ka1[0] * np.dot(n, ka1)
    b[1] = -ka1[0] * np.dot(o, ka1)
    b[2] = -ka1[0] * np.dot(a, ka1)
    b[3] = -ka1[1] * np.dot(n, ka1)
    b[4] = -ka1[1] * np.dot(o, ka1)
    b[5] = -ka1[1] * np.dot(a, ka1)
    b[6] = -ka1[2] * np.dot(n, ka1)
    b[7] = -ka1[2] * np.dot(o, ka1)
    b[8] = -ka1[2] * np.dot(a, ka1)
    
    return A, b

def skew(v):
    return np.array([[0, -v[2], v[1]],
                    [v[2], 0, -v[0]],
                    [-v[1], v[0], 0]])




def calibrate_calculate(_T_0_rob_catted, _T_cam_rigid_catted, save_path = None):
    '''
    args:
        _T_0_rob_catted: 4*4n 矩阵 机械臂末端位姿
        _T_cam_rigid_catted: 4*4n 矩阵 固定相机坐标系下机械臂末端固连刚体的位姿
        save_path: str 如不需要保存最终结果，则None
    return:
        X: 4*4 T_0_camera  机械臂基坐标系下camera的位姿
    '''
    

    if _T_0_rob_catted.shape != _T_cam_rigid_catted.shape:
        print(f'calibrate_calculate  传入矩阵形状不同：{_T_0_rob_catted.shape} != {_T_cam_rigid_catted.shape}')
        return

    n = _T_0_rob_catted.shape[1] // 4
    A = np.zeros((4,4*(n-1)))
    B = np.zeros((4,4*(n-1)))
    for i in range(n-1):
        A[:,4*i:4*i+4] = _T_0_rob_catted[:,4*(i+1):4*(i+1)+4] @ np.linalg.inv(_T_0_rob_catted[:,4*i:4*i+4])
        B[:,4*i:4*i+4] = _T_cam_rigid_catted[:,4*(i+1):4*(i+1)+4] @ np.linalg.inv(_T_cam_rigid_catted[:,4*i:4*i+4])

    X = shiu(A, B)
    if save_path is not None:
        print(f'T_0_camera 保存位置：{save_path}')
        np.savetxt(save_path, X, delimiter=',')
    return X