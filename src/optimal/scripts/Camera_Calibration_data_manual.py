import urx
import math3d as m3d
import numpy as np
import time
import cv2 as cv
import threading
from threading import Lock,Thread
import time,os

import signal
import sys
import select
import tty
import termios

Capturing = 0
Capture_stop = 0
step_i = 0

# #这些变换矩阵只是事先估计的，便于生成机械臂末端位姿作为采样点 ---------------------------------------------------------
# T_shaft_camera = np.array([ [   1,          0,                      0,                      0],
#                             [   0,          np.cos(np.pi/6),        np.sin(np.pi/6),        0],
#                             [   0,          -np.sin(np.pi/6),       np.cos(np.pi/6),        0],
#                             [   0,          0,                      0,                      1]])
# T_camera_shaft = np.linalg.inv(T_shaft_camera)

# T_robot_shaft = np.array([  [   -1,         0,          0,        0     ],
#                             [   0,          0,          1,        0.21  ],
#                             [   0,          1,          0,        0.035 ],
#                             [   0,          0,          0,        1     ]])
# T_shaft_robot = np.linalg.inv(T_robot_shaft)

# T_robot_camera = T_robot_shaft @ T_shaft_camera
# T_camera_robot = np.linalg.inv(T_robot_camera)

# T_robot = np.identity(4) #robot base 坐标系下机械臂末端位姿变换矩阵，只有一方的T默认为robot base 坐标系该方的位姿

#                     # np.array([  [   0,                  -1,     0,                  x],
#                     #             [   -np.cos(theta),     0,      np.sin(theta),      y],
#                     #             [   -np.sin(theta),     0,      -np.cos(theta),     z],
#                     #             [   0,                  0,      0,                  1]])
# # ------------------------------------------------------------------------------------------------------------

def keyboard_interrupt(signal, frame):
    print("Keyboard Interrupt detected!")
    raise KeyboardInterrupt 



            



if __name__ == '__main__':

    cap = cv.VideoCapture(0)
    
    # cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # cap.set(cv.CAP_PROP_FPS, 30)
    # cap.set(4, 1080)  # 图片宽度
    # cap.set(3, 1920)  # 图片宽度
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    cap.set(cv.CAP_PROP_FPS, 30)
    cap.set(4, 720)  # 图片宽度
    cap.set(3, 1280)  # 图片宽度
    cv.namedWindow('figure', 0)
    # cv.resizeWindow('figure', 960, 540)



    img_path = f'{os.path.dirname(__file__)}/../data/Camera_Calibration/imgs/'
    T_path = f'{os.path.dirname(__file__)}/../data/Camera_Calibration/RobotPose.csv'

    rob = urx.Robot("192.168.100.102")
    rob.set_tcp((0, 0, 0, 0, 0, 0)) 
    rob.set_payload(2, (0, 0, 0.1))
    time.sleep(0.2)


    Note=open(T_path,mode='a')
    Note.truncate(0)


    signal.signal(signal.SIGINT, keyboard_interrupt)
    print("1.Press Enter to capture \n2.Press Ctrl+C to exit...")
    # 将终端设置为非规范模式,不修改的话需要回车，而且回车也会被读到
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    step_i = 0
    try:
        while True:
            success, frame = cap.read()
            cv.imshow('figure', frame)
            cv.waitKey(1)
            # 检查标准输入是否有可读数据
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist :
                input_data = sys.stdin.read(1)

                print("capturing step: ",step_i)

                trans_read = rob.get_pose()
                pose_read_array = trans_read.array
                for j in range(4):
                    for k in range(4):
                        Note.write(str(pose_read_array[j,k])+',')
                    Note.write('\n')
                
                print('pose_getted:',step_i)

                success, frame = cap.read()
                if not success:
                    print('error!')
                    exit()
                cv.imwrite(img_path+str(step_i).zfill(4)+'.png', frame)
                Capturing = 0
                print('Capture finish i:',step_i)
                step_i += 1


            
            
    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!")
       

    # 恢复终端设置
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("------恢复终端设置-----")

    
    # # trans.orient.rotate_xb(-np.pi/6)
    # print(trans.array)
    # rob.movel((0.05, 0, 0, 0, 0, 0), 0.5, 0.1, relative=True) 
    # rob.my_speedl([0, 0, 0.01, 0, 0, 0],0.5,3)
    rob.close()
    Capture_stop = 1
    Note.close()
    print('program closed')


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

