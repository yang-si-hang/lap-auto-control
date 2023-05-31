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
i = 0

T_robot_tool = np.identity(4)
                    # np.array([  [   0,                  -1,     0,                  x],
                    #             [   -np.cos(theta),     0,      np.sin(theta),      y],
                    #             [   -np.sin(theta),     0,      -np.cos(theta),     z],
                    #             [   0,                  0,      0,                  1]])
T_tool_robot = np.linalg.inv(T_robot_tool)


class Camera_Thread(threading.Thread):

    def __init__(self,n):
        super(Camera_Thread,self).__init__()   #重构run函数必须写
        self.n = n
        self.cap = cv.VideoCapture(2)

    def run(self):
        global Capturing,Capture_stop,i
        while(1):
            success, frame = self.cap.read()
            if(Capturing):
                print('Capture start:',i)
                success, frame = self.cap.read()
                if not success:
                    print('error!')
                    exit()
                cv.imwrite(img_path+str(i).zfill(4)+'.png', frame)
                Capturing = 0
                print('Capture finish i:',i)
            if(Capture_stop):
                break

if __name__ == '__main__':

    T1=Camera_Thread('T1')
    T1.start()

    N = 100
    theta0 = np.pi/6
    theta = theta0
    radius = 0.6
    x = 0.45
    z = radius*np.cos(theta0)
    y = -radius*np.sin(theta0)


    img_path = f'{os.path.dirname(__file__)}/../../data/Camera_Calibration/imgs/'
    T_path = f'{os.path.dirname(__file__)}/../../data/Camera_Calibration/RobotPose.csv'

    rob = urx.Robot("192.168.100.102")
    rob.set_tcp((0, 0, 0, 0, 0, 0)) 
    rob.set_payload(2, (0, 0, 0.1))
    time.sleep(0.2)


    Note=open(T_path,mode='a')
    Note.truncate(0)

    for i in range(N+1):
        theta = theta0 - i*theta0*2/N
        z = radius*np.cos(theta)
        y = -radius*np.sin(theta)
        pos_array = np.array([  [   0,                  -1,     0,                  x],
                                [   -np.cos(theta),     0,      np.sin(theta),      y],
                                [   -np.sin(theta),     0,      -np.cos(theta),     z],
                                [   0,                  0,      0,                  1]])
        trans_set = m3d.Transform(pos_array)
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

