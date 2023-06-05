# coding=utf-8
# instrument pos 根据器械上的imu读到倾角和图像中器械末端位置坐标估计器械末端三维位置

#能不能把instrument相关的变量和函数封装个类啊，这01 lr的，依托狮山

import copy
import sys
import logging
import os.path
import time

import numpy as np
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
# from skimage.draw import line
from scipy.optimize import fsolve, brentq, root
from spatialmath.base import *
from math3d.transform import Transform as Trans
# from my_pkg.dual_ur5_kin import LeftUr5, RightUr5

# sys.path.append("/home/yiliao/wyh/IMU_drivers/openzenros_ws/devel/lib")
# import openzen
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R

import torchvision
import torch
sys.path.append(f'{os.path.dirname(__file__)}/../wyhyolo')
from wyhyolo.models.common import DetectMultiBackend
from wyhyolo.miniyolo import get_box
from wyhyolo.detect import select_device



np.set_printoptions(precision=3,suppress=True)

path = os.path.dirname(__file__)

IntrinsicMatrix = np.loadtxt(f'{path}/../data/mtx.csv')
IntrinsicMatrix_inv = np.linalg.inv(IntrinsicMatrix)
camera_tool = np.loadtxt(f'{path}/../data/camera_tool.csv')
dist = np.loadtxt(f'{path}/../data/dist.csv')


q0 = np.pi/6

# left_tip_0 = np.array([0.55, 0.08, 0.93])
# right_tip_0 = np.array([0.45, 0.08, 0.93])
left_tip_0 = np.array([0.50975898, 0.30078158, 0.05010113])
right_tip_0 = np.array([0.61339974, 0.31193065, 0.05948734])
beta = 0.9      # 移动加权平均系数

rcm_l_4d_base = np.array([0.45108569, 0.26102381, 0.16898061, 1.])
rcm_r_4d_base = np.array([0.67721158, 0.25670649, 0.17017894, 1.])
rcm_l_base = np.array(rcm_l_4d_base[0:3])
rcm_r_base = np.array(rcm_r_4d_base[0:3])
rcm_l_4d_c = None
rcm_r_4d_c = None


# rcm_l_4d_base = np.array([0.59508, 0.22294, 0.99971, 1.])
# rcm_r_4d_base = np.array([0.43117, 0.22188, 1.00401, 1.])

# left_base = np.array([[-0.9981, 0.06149, -0.004677, 0.01738],
#                       [0.04068, 0.7135, 0.6995, 0.2161],
#                       [0.04635, 0.698, -0.7146, 1.534],
#                       [0, 0, 0, 1]])
# left_base = left_base @ trotz(-np.pi)
# left_base_inv = np.linalg.inv(left_base)
right_base = np.identity(4)
# right_base = right_base @ trotz(-np.pi)
right_base_inv = np.linalg.inv(right_base)
# camera_robot = LeftUr5()
# joint = np.array([[31.68, -106.71, -106.11, -44.32, 104.90, -146.69]])
# joint = np.deg2rad(joint)
# T_0_c = camera_robot.my_fkine(joint)
# T_0_c = T_0_c.A @ camera_tool

# T_0_c = np.identity(4)
T_0_c = None


# imu0 -> instrument_left -----------------------
imu0_quaternionre_read = np.array([0, 0, 0, 1])
q0_inited_flag = 0
q0_base = np.array([0, 0, 0, 1])
R_imu00_base = np.identity(3)
R_base_imu00 = np.identity(3)
R_imu00_imu0 = np.identity(3)
R_base_imu0 = np.identity(3)

#imu1 -> instrument_right -----------------------
imu1_quaternionre_read = np.array([0, 0, 0, 1])
q1_inited_flag = 0
q1_base = np.array([0, 0, 0, 1])
R_imu10_base = np.identity(3)
R_base_imu10 = np.identity(3)
R_imu10_imu1 = np.identity(3)
R_base_imu1 = np.identity(3)




inst_l_vector = None #左器械轴向在rob base 坐标系下的单位方向向量
inst_r_vector = None 
inst_l_c_vector = None #左器械末端在rob base 坐标系下，camera视线得出的单位方向向量
inst_r_c_vector = None 
inst_boxs = None
inst_box_l = []  #涉及左右器械的编号，0为左，1为右，box中的标签值如此
inst_box_r = []
inst_l_pos = None
inst_r_pos = None




def ShaftPoseProcess(msg):
    global T_0_c
    T_b_s = Trans.get_array(Trans(msg.data))
    T_0_c = right_base @ T_b_s @ trotx(-q0)

    # print(T_0_c)

def imu_r_read(msg):
    global imu1_quaternionre_read
    global R_imu10_imu1
    global R_imu10_base
    global R_base_imu10
    global R_base_imu1
    global inst_r_vector

    imu1_quaternionre_read = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    # print('imu1 data:')
    # print(imu1_quaternionre_read)

    
    
    R_imu10_imu1 = R.from_quat(imu1_quaternionre_read).as_matrix()
    R_base_imu1 = R_base_imu10 @ R_imu10_imu1
    inst_r_vector = np.array(R_base_imu1[0:3][0])  #取imu1的x轴与器械轴对齐
    pass



def cross_point_pos(inst_index):
    global T_0_c

    global inst_l_vector
    global inst_l_c_vector
    global R_base_imu0
    global inst_l_pos

    global inst_r_vector
    global inst_r_c_vector
    global R_base_imu1
    global inst_r_pos
    
    if inst_index == 0:
        inst_l_c_vector = IntrinsicMatrix_inv @ np.array([ (inst_box_l[0]+inst_box_l[2])/2,    (inst_box_l[1]+inst_box_l[3])/2,   1])
        inst_l_c_vector = T_0_c[0:3,0:3] @ inst_l_c_vector
        inst_l_vector = np.squeeze(R_base_imu1[0:3,0])
        d_vector = np.cross( inst_l_vector, inst_l_c_vector )
        d_vector = d_vector/np.linalg.norm(d_vector)
        vector_c_rcm = rcm_l_base - np.squeeze(T_0_c[0:3,3])
        d_vector = np.dot(vector_c_rcm, d_vector) * d_vector
        k = (T_0_c[1,3] + d_vector[1] - rcm_l_base[1])*inst_l_c_vector[0] + (rcm_l_base[0] - T_0_c[0,3] - d_vector[0])*inst_l_c_vector[1]
        k = k/(inst_l_vector[1] * inst_l_c_vector[0] - inst_l_vector[0] * inst_l_c_vector[1])
        inst_l_pos = k * inst_l_vector + rcm_l_base

    
    if inst_index == 1:
        inst_r_c_vector = IntrinsicMatrix_inv @ np.array([ (inst_box_r[0]+inst_box_r[2])/2,    (inst_box_r[1]+inst_box_r[3])/2,   1])
        inst_r_c_vector = T_0_c[0:3,0:3] @ inst_r_c_vector
        inst_r_vector = np.squeeze(R_base_imu1[0:3,0])
        d_vector = np.cross( inst_r_vector, inst_r_c_vector )
        d_vector = d_vector/np.linalg.norm(d_vector)
        vector_c_rcm = rcm_r_base - np.squeeze(T_0_c[0:3,3])
        d_vector = np.dot(vector_c_rcm, d_vector) * d_vector
        # x-y
        # k = (T_0_c[1,3] + d_vector[1] - rcm_r_base[1])*inst_r_c_vector[0] + (rcm_r_base[0] - T_0_c[0,3] - d_vector[0])*inst_r_c_vector[1]
        # k = k/(inst_r_vector[1] * inst_r_c_vector[0] - inst_r_vector[0] * inst_r_c_vector[1])
        # y-z
        k = (T_0_c[1,3] + d_vector[1] - rcm_r_base[1])*inst_r_c_vector[2] + (rcm_r_base[2] - T_0_c[2,3] - d_vector[2])*inst_r_c_vector[1]
        k = k/(inst_r_vector[1] * inst_r_c_vector[2] - inst_r_vector[2] * inst_r_c_vector[1])
        inst_r_pos = k * inst_r_vector + rcm_r_base
        print("d vector: ",d_vector,'  norm: ',np.linalg.norm(d_vector))
        # print("inst c vector: ",inst_r_c_vector)
        # print("inst rcm vector: ",inst_r_vector)
        print("inst end pos:\n\t",inst_r_pos,'\n\t',(inst_r_pos-d_vector),'\n\t',(inst_r_pos-d_vector*0.5))



if __name__ == '__main__':
    rospy.init_node('inst_pos_imu_yolo', anonymous=True)

    # 读取 imu 的初始化数据
    print("reading imu q base......")
    q1_base = np.loadtxt(f'{os.path.dirname(__file__)}/../data/init_q1_base.csv', delimiter=",")
    R_imu10_base = R.from_quat(q1_base).as_matrix()
    R_base_imu10 = R_imu10_base.T
    print("q1_base readed: ",q1_base)

    #注册订阅者和发布者，注意保证imu初始化数据已读取，以确保后续读取imu数据的回调计算正常进行
    left_pub = rospy.Publisher('LeftPos', numpy_msg(Floats), queue_size=1)
    right_pub = rospy.Publisher('RightPos', numpy_msg(Floats), queue_size=1)
    rospy.Subscriber('ShaftPose', numpy_msg(Floats), ShaftPoseProcess)
    rospy.Subscriber("/imu_r/imu/data",Imu,imu_r_read,queue_size=1)

    cv2.namedWindow('figure', 0)
    cv2.resizeWindow('figure', 960, 540)
    capture = cv2.VideoCapture(0)
    # v4l2 设置摄像头 https://blog.csdn.net/lantian510/article/details/119395673
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    capture.set(cv2.CAP_PROP_FPS, 30)
    # # # print(f'亮度：{capture.get(10)}')
    # # # print(f'对比度：{capture.get(11)}')
    # # # print(f'增益：{capture.get(14)}')
    capture.set(10, 200)  # 亮度
    # # capture.set(14, 4)   # 增益        
    # # # capture.set(14, 8)   # 增益
    # # capture.set(15, 80)   # 曝光度
    # # # print(f'亮度：{capture.get(10)}')
    # # # capture.set(11, 37)
    # # capture.set(4, 720)  # 图片宽度
    # # capture.set(3, 1280)  # 图片宽度
    capture.set(4, 1080)  # 图片宽度
    capture.set(3, 1920)  # 图片宽度
    # # capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    # fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')

    DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')  # 放在cuda或者cpu上训练
    # device = 'cuda:0'
    # device = 'cpu'
    # device = None
    # select_device(device)
    # print('device: ',device)
    model = DetectMultiBackend(f'{path}/wyhyolo/run/train/wyh/weights/best.pt', device=DEVICE, dnn=False)
    model.warmup()

    
    

    # 等待 ShaftPose 的数据
    if T_0_c is None:
        print('T_0_c is None')
    while T_0_c is None and  not rospy.is_shutdown():
        pass


    # 主循环
    while  not rospy.is_shutdown():
        print('------------------------------------------------------------------')
        start0 = time.time()
        # if T_0_c is None:
        #     time.sleep(0.05)
        #     continue
        rcm_l_4d_c = np.linalg.inv(T_0_c) @ rcm_l_4d_base  #camera 坐标系下
        rcm_l_4d_c = np.squeeze(rcm_l_4d_c)
        rcm_r_4d_c = np.linalg.inv(T_0_c) @ rcm_r_4d_base
        rcm_r_4d_c = np.squeeze(rcm_r_4d_c)


        start1 = time.time()
        ret, frame = capture.read()
        total_time = time.time() - start1
        print("capture shape :",frame.shape)
        print(f'capture time:{total_time} s    {1./total_time} Hz')

        # frame = cv2.imread('/home/wyh/project/laparoscope_ws/src/optimal/scripts/wyhyolo/data/images/00378.jpg')
        # frame = cv2.imread('/home/wyh/project/laparoscope_ws/src/optimal/scripts/wyhyolo/data/images/00015.jpg')



        start_time = time.time()
        inst_boxs = get_box(my_model=model, img0=frame, device=DEVICE)
        # print ("inst boxs: ",inst_boxs)
        print("box time : ", time.time()-start_time)
        


        if len(inst_boxs)==2:
            if inst_boxs[0][-1] == 0:
                inst_box_l = inst_boxs[0]
                inst_box_r = inst_boxs[1]
            else:
                inst_box_l = inst_boxs[1]
                inst_box_r = inst_boxs[0]
            # inst_box_l[0] = inst_box_l[0] * 3
            # inst_box_r[0] = inst_box_r[0] * 3
            # inst_box_l[2] = inst_box_l[2] * 3
            # inst_box_r[2] = inst_box_r[2] * 3
            # inst_box_l[1] = inst_box_l[1] * 2.75
            # inst_box_r[1] = inst_box_r[1] * 2.75
            # inst_box_l[3] = inst_box_l[3] * 2.75
            # inst_box_r[3] = inst_box_r[3] * 2.75
        else:
            if len(inst_boxs) ==1:
                if inst_boxs[0][-1] == 0:
                    inst_box_l = inst_boxs[0]
                else:
                    inst_box_r = inst_boxs[0]


        # print('box detect cost: ',time.time()-start_time,' s')
        # print('inst_box_l: ',inst_box_l)
        # print('inst_box_r: ',inst_box_r)
        





        if len(inst_box_l)  != 0:
            cross_point_pos(0)
            point1 =  ((inst_box_l[0]).astype(np.int32),    (inst_box_l[1]).astype(np.int32))
            point2 =  ((inst_box_l[2]).astype(np.int32),    (inst_box_l[3]).astype(np.int32))
            cv2.rectangle(frame, point1, point2, (0, 255, 0), thickness=1)
            # left_pub.publish( inst_l_pos.astype(np.float32) )

        if len(inst_box_r) != 0:
            cross_point_pos(1)
            point1 =  ((inst_box_r[0]).astype(np.int32),    (inst_box_r[1]).astype(np.int32))
            point2 =  ((inst_box_r[2]).astype(np.int32),    (inst_box_r[3]).astype(np.int32))
            cv2.rectangle(frame, point1, point2, (255, 0, 0), thickness=1)
            right_pub.publish( inst_r_pos.astype(np.float32) )
            inst_l_pos = inst_r_pos - np.array([ 0.04,  0., 0.])
            left_pub.publish( inst_l_pos.astype(np.float32) )


        frame = cv2.resize(frame, (1280,720))


        


    

        cv2.imshow('figure', frame)
        # outCamera.write(img_show)
        cv2.waitKey(1)
        # print(time.time()-start4)
        total_time = time.time() - start0
        print(f'total time:{total_time} s    {1./total_time} Hz')


