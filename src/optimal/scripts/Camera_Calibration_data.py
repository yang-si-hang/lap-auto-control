import urx
import math3d as m3d
import numpy as np
import time
import cv2 as cv
import threading
from threading import Lock,Thread
import time,os


Capturing = 0
Capture_stop = 0
step_i = 0


T_shaft_camera = np.array([ [   1,          0,                      0,                      0],
                            [   0,          np.cos(np.pi/6),        np.sin(np.pi/6),        0],
                            [   0,          -np.sin(np.pi/6),       np.cos(np.pi/6),        0],
                            [   0,          0,                      0,                      1]])
T_camera_shaft = np.linalg.inv(T_shaft_camera)

T_robot_shaft = np.array([  [   -1,         0,          0,        0     ],
                            [   0,          0,          1,        0.21  ],
                            [   0,          1,          0,        0.035 ],
                            [   0,          0,          0,        1     ]])
T_shaft_robot = np.linalg.inv(T_robot_shaft)

T_robot_camera = T_robot_shaft @ T_shaft_camera
T_camera_robot = np.linalg.inv(T_robot_camera)

T_robot = np.identity(4)

                    # np.array([  [   0,                  -1,     0,                  x],
                    #             [   -np.cos(theta),     0,      np.sin(theta),      y],
                    #             [   -np.sin(theta),     0,      -np.cos(theta),     z],
                    #             [   0,                  0,      0,                  1]])



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

    N = 50
    theta01 = np.pi/8
    theta02 = np.pi/4
    theta = theta01
    radius = 0.52
    x_center = 0.52
    x = x_center
    z = radius*np.cos(theta01)
    y = 0


    img_path = f'{os.path.dirname(__file__)}/../data/Camera_Calibration/imgs/'
    T_path = f'{os.path.dirname(__file__)}/../data/Camera_Calibration/RobotPose.csv'

    rob = urx.Robot("192.168.100.102")
    rob.set_tcp((0, 0, 0, 0, 0, 0)) 
    rob.set_payload(2, (0, 0, 0.1))
    time.sleep(0.2)


    Note=open(T_path,mode='a')
    Note.truncate(0)



    for i in range(N+1):
        step_i = i
        theta = theta01 - i*theta01/N
        z = radius*np.cos(theta)
        y = 0
        x = x_center
        T_camera = np.array([  [   np.cos(theta),      0,      -np.sin(theta),     x],
                                [   0,                  -1,     0,                  y],
                                [   -np.sin(theta),     0,      -np.cos(theta),     z],
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


    for i in range(N+1):
        step_i = N+1+i
        theta = i*theta02/N
        z = radius*np.cos(theta)
        y = -radius*np.sin(theta)
        x = x_center
        T_camera = np.array([   [   1,      0,                  0,                  x],
                                [   0,      -np.cos(theta),     np.sin(theta),      y],
                                [   0,      -np.sin(theta),     -np.cos(theta),     z],
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


#===================================  读取矩阵txt  ============================================
    # A=np.zeros((4*N+4,4),dtype=float) #先创建全零矩阵A,并将数据设置为float类型
    # f=open(T_path)
    # lines=f.readlines() #将全部数据读到一个lines中
    # A_row=0         #表示矩阵的行，从0开始
    # for line in lines:
    #     list=line.strip('\n').split(',')
    #     A[A_row:]=list[0:4]
    #     A_row += 1

    # print(A)
#===============================================================================================

