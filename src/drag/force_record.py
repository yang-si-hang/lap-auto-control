"""
2025.3.6
该程序不用于动物实验等配套使用，只作为单独运行的小程序

"""

import os.path
from datetime import datetime

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int8
import time
import logging
import sys
from spatialmath.base import trotx, troty, trotz, transl, angvec2tr, rpy2tr
from math3d.transform import Transform as Trans
from scipy.spatial.transform import Rotation
import math3d as m3d
import random
from geometry_msgs.msg import TwistStamped, Point
from geometry_msgs.msg import WrenchStamped, Wrench

np.set_printoptions(precision=6, suppress=True)

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src")
import drag.force_sensor_receiver as force_sensor_receiver

sys.path.append(f"{os.path.dirname(__file__)}/../../scripts/my_tools")
import key_signal

sys.path.append(f"{os.path.dirname(__file__)}/../../scripts")
import rokae_basic_fun 

import threading

data_folder = "/home/irobotcare/桌面/EX_Data/其他测试/drag"



def record_original_force():
    now = datetime.now()
    # 格式化时间为 'YYYY-MM-DD HH:MM:SS' 格式
    formatted_time = now.strftime("%Y%m%d%H%M%S")
    file_name = f"{data_folder}/原始力{formatted_time}.txt"
    data_list=[]

    def force_sensor_callback(msg):
        time_stamp = time.perf_counter()
        force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        torque = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        F = force + torque
        data = [time_stamp]+F
        data_list.append(data) 
        # print(data)

    rospy.Subscriber('/Bota_force_sensor/wrenchstamped',WrenchStamped,  force_sensor_callback)
    try:
        print("record start")
        while not rospy.is_shutdown():
            pass
            
    except:
        print("record_original_force: except")
        
    finally:
        print("record_original_force: finally")
        np.savetxt(file_name,np.array(data_list),delimiter=',')

def record_pure_force(F_sensor, rokae):
    now = datetime.now()
    # 格式化时间为 'YYYY-MM-DD HH:MM:SS' 格式
    formatted_time = now.strftime("%Y%m%d%H%M%S")
    file_name = f"{data_folder}/操作力{formatted_time}.txt"
    data_list=[]
    R_rob_sensor = lap_set.T_rob_sensor[:3,:3]

    try:
        print("record start")
        while not rospy.is_shutdown():
            time_stamp = time.perf_counter()
            R_0_rob = rokae.pose.R_matrix()
            R_0_sensor = R_0_rob @ R_rob_sensor
            F_pure = F_sensor.pure_force_now(R_0_sensor)
            data = np.insert(F_pure,0,time_stamp)
            data_list.append(data.copy())
            print(data)
            pass
            
    except:
        print("record_original_force: except")
        
    finally:
        print("record_original_force: finally")
        np.savetxt(file_name,np.array(data_list),delimiter=',')


if __name__ == "__main__":
    rospy.init_node("force_record")
    F_sensor = force_sensor_receiver.force_sensor_receiver_class()
    rokae = rokae_basic_fun.rokae()
    # record_original_force()
    record_pure_force(F_sensor, rokae)
    pass