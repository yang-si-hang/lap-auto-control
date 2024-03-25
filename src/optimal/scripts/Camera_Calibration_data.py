'''
按照轨迹自动收集标定图片和机械臂位姿，存储位置见 line 114
记得提前将存储位置老文件清空（不需要清空了，变换矩阵直接覆盖了）
要清空： ../data/Camera_Calibration/imgs/  （如果图片数量一样会直接覆盖，原有多的不会覆盖） 
不进行运算，运算在
./Camera_Calibration_calculate.py

需要修改的部分：
    1、估计相机和机械臂末端关系的变换矩阵，不改变夹具、镜头、末端安装方向，则不需要更改。
    2、xyz的中心位置和s初始位置，根据标定板摆放位置修改。
    3、循环中镜头期望的位姿，不要忘记修改xyz坐标，并且两个循环都要修改。
'''

import urx
import math3d as m3d
import numpy as np
import time
import cv2 as cv
import threading
from threading import Lock,Thread
import time,os
import sys


sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set


Capturing = 0
Capture_stop = 0
step_i = 0

#这些变换矩阵只是事先估计的，便于生成机械臂末端位姿作为采样点 ---------------------------------------------------------
T_shaft_camera = np.array([ [   1,          0,                      0,                      0],
                            [   0,          np.cos(np.pi/6),        np.sin(np.pi/6),        0],
                            [   0,          -np.sin(np.pi/6),       np.cos(np.pi/6),        0],
                            [   0,          0,                      0,                      1]])
T_camera_shaft = np.linalg.inv(T_shaft_camera)

T_robot_shaft = np.array([  [   -1,         0,          0,        0     ],
                            [   0,          0,          1,        0.21  ],
                            [   0,          1,          0,        0.035 ],
                            [   0,          0,          0,        1     ]])#小腹腔镜的参数
T_shaft_robot = np.linalg.inv(T_robot_shaft)

T_robot_camera = T_robot_shaft @ T_shaft_camera #镜头相对机械臂末端（考虑了镜头斜角）
T_camera_robot = np.linalg.inv(T_robot_camera)

T_robot = np.identity(4) #robot base 坐标系下机械臂末端位姿变换矩阵，只有一方的T默认为robot base 坐标系该方的位姿

                    # np.array([  [   0,                  -1,     0,                  x],
                    #             [   -np.cos(theta),     0,      np.sin(theta),      y],
                    #             [   -np.sin(theta),     0,      -np.cos(theta),     z],
                    #             [   0,                  0,      0,                  1]])
# ------------------------------------------------------------------------------------------------------------


class Camera_Thread(threading.Thread):

    def __init__(self,n):
        super(Camera_Thread,self).__init__()   #重构run函数必须写
        self.n = n
        self.cap = cv.VideoCapture(0)
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv.CAP_PROP_FPS, 30)
        self.cap.set(4, 1080)  # 图片宽度
        self.cap.set(3, 1920)  # 图片宽度

    def run(self):
        global Capturing,Capture_stop,step_i
        while(1):
            success, frame = self.cap.read()
            if(Capturing):
                print('Capture start:',step_i)
                success, frame = self.cap.read()
                if not success:
                    print('error!')
                    exit()
                cv.imwrite(img_path+str(step_i).zfill(4)+'.png', frame)
                Capturing = 0
                print('Capture finish i:',step_i)
            if(Capture_stop):
                break



if __name__ == '__main__':

    T1=Camera_Thread('T1')
    T1.start()

    # ====================================================================================================
    

    #2023.11.14 ur10e-------------------------------------------------------------------------------------
    '''
    后面的循环中也要修改xyz位置
    '''
    N = 50
    theta01 = np.pi/4
    theta02 = np.pi/5
    theta = theta01
    radius = 0.25
    x_center = 0
    y_center = -0.7
    z_center = 0
    x = x_center
    y = y_center
    z = radius*np.cos(theta01)






    img_path = f'{os.path.dirname(__file__)}/../data/Camera_Calibration/imgs/'
    T_path = f'{os.path.dirname(__file__)}/../data/Camera_Calibration/RobotPose.csv'

    rob = urx.Robot(lap_set.robot_ip)
    rob.set_tcp((0, 0, 0, 0, 0, 0)) 
    rob.set_payload(0, (0, 0, 0.1))
    time.sleep(0.2)


    Note=open(T_path,mode='a') #追加模式打开，配合下一行命令
    Note.truncate(0)  #清空原有内容


# ================================================================================================================
    # 从最远点到顶点，非对称，单侧圆弧  theta01 -> 0
    # for i in range(N+1):
    #     step_i = i
    #     theta = theta01 - i*theta01/N
    #     z = radius*np.cos(theta)
    #     y = 0
    #     x = x_center
    #     T_camera = np.array([  [   np.cos(theta),      0,      -np.sin(theta),     x],
    #                             [   0,                  -1,     0,                  y],
    #                             [   -np.sin(theta),     0,      -np.cos(theta),     z],
    #                             [   0,                  0,      0,                  1]])
    #     T_robot = T_camera @ T_camera_robot
    #     trans_set = m3d.Transform(T_robot)
    #     print('step i:',i)
    #     rob.set_pose(trans_set, acc=0.5, vel=0.2)
    #     trans_read = rob.get_pose()
    #     pose_read_array = trans_read.array
    #     for j in range(4):
    #         for k in range(4):
    #             Note.write(str(pose_read_array[j,k])+',')
    #         Note.write('\n')
        
    #     print('pose_getted:',i)
    #     time.sleep(0.02)
    #     Capturing = 1
    #     time.sleep(0.2)

    #由顶端远离 0 -> theta02
    # for i in range(N+1):
    #     step_i = N+1+i
    #     theta = i*theta02/N
    #     z = radius*np.cos(theta)
    #     y = -radius*np.sin(theta)
    #     x = x_center
    #     T_camera = np.array([   [   1,      0,                  0,                  x],
    #                             [   0,      -np.cos(theta),     np.sin(theta),      y],
    #                             [   0,      -np.sin(theta),     -np.cos(theta),     z],
    #                             [   0,                  0,      0,                  1]])
    #     T_robot = T_camera @ T_camera_robot
    #     trans_set = m3d.Transform(T_robot)
    #     print('step i:',i)
    #     rob.set_pose(trans_set, acc=0.5, vel=0.2)
    #     trans_read = rob.get_pose()
    #     pose_read_array = trans_read.array
    #     for j in range(4):
    #         for k in range(4):
    #             Note.write(str(pose_read_array[j,k])+',')
    #         Note.write('\n')
        
    #     print('pose_getted:',i)
    #     time.sleep(0.02)
    #     Capturing = 1
    #     time.sleep(0.2)

    # 2023.11.14,. ------------------------------------------------------------------------------------------------------------
    # 从最远点到顶点，非对称，单侧圆弧  theta01 -> 0
    for i in range(N+1):
        step_i = i
        theta = theta01 - i*theta01/N
        # 需要修改的参数...............................................................................
        z = radius*np.cos(theta)
        y = y_center - radius * np.sin(theta)
        x = x_center
        # 期望的相机镜头位姿
        T_camera = np.array([   [   1,      0,                  0,                  x],
                                [   0,     -np.cos(theta),      np.sin(theta),      y],
                                [   0,      -np.sin(theta),     -np.cos(theta),     z],
                                [   0,      0,                  0,                  1]])
        #...........................................................................................
        T_robot = T_camera @ T_camera_robot
        trans_set = m3d.Transform(T_robot)
        print('step i:',i)
        print('trans_set:\n',trans_set)
        rob.set_pose(trans_set, acc=0.5, vel=0.2)
        time.sleep(0.2)
        trans_read = rob.get_pose()
        pose_read_array = trans_read.array
        for j in range(4):
            for k in range(4):
                Note.write(str(pose_read_array[j,k])+',')
            Note.write('\n')
        
        print('pose_getted:',i)
        time.sleep(0.02)
        Capturing = 1
        time.sleep(0.2)

    #由顶端远离 0 -> theta02
    for i in range(N+1):
        step_i = N+1+i
        theta = i*theta02/N
        z = radius*np.cos(theta)
        y = y_center
        x = x_center - radius*np.sin(theta)
        T_camera = np.array([   [   np.cos(theta),      0,      np.sin(theta),      x],
                                [   0,                  -1,     0,                  y],
                                [   np.sin(theta),      0,      -np.cos(theta),     z],
                                [   0,                  0,      0,                  1]])
        T_robot = T_camera @ T_camera_robot
        trans_set = m3d.Transform(T_robot)
        print('step i:',i)
        rob.set_pose(trans_set, acc=0.5, vel=0.2)
        trans_read = rob.get_pose()
        pose_read_array = trans_read.array
        for j in range(4):
            for k in range(4):
                Note.write(str(pose_read_array[j,k])+',')
            Note.write('\n')
        
        print('pose_getted:',i)
        time.sleep(0.02)
        Capturing = 1
        time.sleep(0.2)


    # # trans.orient.rotate_xb(-np.pi/6)
    # print(trans.array)
    # rob.movel((0.05, 0, 0, 0, 0, 0), 0.5, 0.1, relative=True) 
    # rob.my_speedl([0, 0, 0.01, 0, 0, 0],0.5,3)
    rob.close()
    Capture_stop = 1
    Note.close()



