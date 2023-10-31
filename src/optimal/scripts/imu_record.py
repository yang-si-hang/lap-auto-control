
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





from spatialmath.base import *


file_name = '/home/yiliao/870evo_1t/Experiment_Data/20230629/imu.txt'

#imu1 -> instrument_right -----------------------
imu1_quaternionre_read = np.array([0, 0, 0, 1])
q1_inited_flag = 0
q1_base = np.array([0, 0, 0, 1])
R_imu10_base = np.identity(3)
R_base_imu10 = np.identity(3)
R_imu10_imu1 = np.identity(3)
R_base_imu1 = np.identity(3)

def imu_r_read(msg):
    global imu1_quaternionre_read
    global R_imu10_imu1
    global R_imu10_base
    global R_base_imu10
    global R_base_imu1
    global inst_r_vector

    time_stamp = time.time()
    imu1_quaternionre_read = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    # print('imu1 data:')
    # print(imu1_quaternionre_read)

    
    
    R_imu10_imu1 = R.from_quat(imu1_quaternionre_read).as_matrix()
    R_base_imu1 = R_base_imu10 @ R_imu10_imu1
    # inst_r_vector = np.array(R_base_imu1[0:3][0])  #取imu1的x轴与器械轴对齐
    q = r2q(R_base_imu1)
    print(f'{time_stamp}\nR:\n{R_base_imu1}\nq: {q}')

    data = f'{time_stamp},\t{q[0]},{q[1]},{q[2]},{q[3]}\n'
    file.write(data)
    pass


if __name__ == '__main__':

    file = open(file_name, 'a')
    rospy.init_node('imu_record', anonymous=True)

    # 读取 imu 的初始化数据
    print("reading imu q base......")
    q1_base = np.loadtxt(f'{os.path.dirname(__file__)}/../data/init_q1_base.csv', delimiter=",")
    R_imu10_base = R.from_quat(q1_base).as_matrix()
    R_base_imu10 = R_imu10_base.T
    print("q1_base readed: ",q1_base)



    rospy.Subscriber("/imu_r/imu/data",Imu,imu_r_read,queue_size=1)

    while not rospy.is_shutdown():
        pass


    file.close()
    print("imu record finish")
