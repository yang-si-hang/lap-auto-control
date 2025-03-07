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



def record_original_force(F_sensor, fps=500):
    now = datetime.now()
    # 格式化时间为 'YYYY-MM-DD HH:MM:SS' 格式
    formatted_time = now.strftime("%Y%m%d%H%M%S")
    file_name = f"{data_folder}/原始力{formatted_time}.txt"
    file = open(file_name, "w", encoding="utf-8")

    try:
        rate = rospy.Rate(fps)
        while not rospy.is_shutdown():
            time_stamp = time.perf_counter()
            F = F_sensor.F
            data = f"{time_stamp},{F[0]},{F[1]},{F[2]},{F[3]},{F[4]},{F[5]}\n"
            file.write(data)
            print(data)
            rate.sleep()
    except:
        print("record_original_force: except")
        file.close()
    finally:
        print("record_original_force: finally")
        file.close()


if __name__ == "__main__":
    rospy.init_node("force_record")
    F_sensor = force_sensor_receiver.force_sensor_receiver_class()
    record_original_force(F_sensor)
    pass