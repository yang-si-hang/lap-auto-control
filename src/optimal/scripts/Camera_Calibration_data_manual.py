'''
记得提前将存储位置老文件清空（不需要清空了，变换矩阵直接覆盖了）
要清空： img_folder （如果图片数量一样会直接覆盖，原有多的不会覆盖） 
文件路径完全依赖 lap_set

不进行运算，运算在
./Camera_Calibration_calculate.py
'''

# import urx
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
import rospy

sys.path.append(f"{os.path.dirname(__file__)}")
from lap_set_pk import lap_set

sys.path.append(f"{os.path.dirname(__file__)}/../../../scripts")
import rokae_basic_fun 
#=========================================================================================================================

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
    rospy.init_node('camera_calibration_manual')
    cap = cv.VideoCapture(0)
    
    # cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    # cap.set(cv.CAP_PROP_FPS, 30)
    # cap.set(4, 1080)  # 图片宽度
    # cap.set(3, 1920)  # 图片宽度
    cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc('N', 'V', '1', '2'))
    cap.set(cv.CAP_PROP_FPS, 60)
    cap.set(4, 1080)  # 图片宽度
    cap.set(3, 1920)  # 图片宽度
    cv.namedWindow('figure', 0)
    cv.resizeWindow('figure', 960, 540)



    img_folder = f'{lap_set.data_folder}/Camera_Calibration/imgs'
    joints_file = f'{lap_set.data_folder}/Camera_Calibration/rokae_test.txt'
    RobotPose_path = f'{lap_set.data_folder}/Camera_Calibration/RobotPose.csv'

    file = open(joints_file, 'a')
    file.truncate(0)

    rokae = rokae_basic_fun.rokae()

    time.sleep(0.2)


    Note=open(RobotPose_path,mode='a')
    Note.truncate(0)


    signal.signal(signal.SIGINT, keyboard_interrupt) 
    print("1.Press any button to capture \n2.Press Ctrl+C to exit...")
    # 将终端设置为非规范模式,不修改的话需要回车，而且回车也会被读到
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    step_i = 0
    try:
        while True:
            success, frame = cap.read()

            # 将颜色通道的顺序改变为RGR
            if lap_set.rgb2bgr:
                frame = frame[:, :, [2, 1, 0]]

            while frame is None:
                pass
            cv.imshow('figure', frame)
            cv.waitKey(1)
            # 检查标准输入是否有可读数据
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist :
                input_data = sys.stdin.read(1)
               
                print("capturing step: ",step_i)

                # trans_read = rob.get_pose()
                pose_read_array = rokae.pose.T_matrix()
                joints = rokae.JointState.position
                
                joints_str = ', '.join(str(elem) for elem in joints)
                joints_str = f'{joints_str}\n'
                file.write(joints_str)
                for j in range(4):
                    for k in range(4):
                        Note.write(str(pose_read_array[j,k])+',')
                    Note.write('\n')
                
                print('pose_getted:',step_i)

                success, frame = cap.read()

                if not success:
                    print('error!')
                    exit()

                # 将颜色通道的顺序改变为RGR
                if lap_set.rgb2bgr:
                    frame = frame[:, :, [2, 1, 0]]

                cv.imwrite(f'{img_folder}/{str(step_i).zfill(4)}.png', frame)
                Capturing = 0
                print('Capture finish i:',step_i)
                step_i += 1


            
            
    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!")
        file.close()
       

    # 恢复终端设置
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("------恢复终端设置-----")

    
    # # trans.orient.rotate_xb(-np.pi/6)
    # print(trans.array)
    # rob.movel((0.05, 0, 0, 0, 0, 0), 0.5, 0.1, relative=True) 
    # rob.my_speedl([0, 0, 0.01, 0, 0, 0],0.5,3)
    rokae.cp_stop()
    Capture_stop = 1
    Note.close()
    print('program closed')


#===================================  读取矩阵txt  ============================================
    # A=np.zeros((4*N+4,4),dtype=float) #先创建全零矩阵A,并将数据设置为float类型
    # f=open(RobotPose_path)
    # lines=f.readlines() #将全部数据读到一个lines中
    # A_row=0         #表示矩阵的行，从0开始
    # for line in lines:
    #     list=line.strip('\n').split(',')
    #     A[A_row:]=list[0:4]
    #     A_row += 1

    # print(A)
#===============================================================================================

