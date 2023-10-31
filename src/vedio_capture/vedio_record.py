
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
# sys.path.append(f'{os.path.dirname(__file__)}/../../optimal/scripts/wyhyolo')
# from wyhyolo.models.common import DetectMultiBackend
# from wyhyolo.miniyolo import get_box
# from wyhyolo.detect import select_device

import threading
import queue


import signal
import sys

import select

import tty
import termios


vedio_file_path = '/home/yiliao/870evo_1t/Experiment_Data/20230629/output_video.avi'
vedio_time_stamp_file_path = '/home/yiliao/870evo_1t/Experiment_Data/20230629/vedio_timestamp.txt'

# 捕获线程的任务函数
def capture_thread(cap, buffer):
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        buffer.put(frame)

# 显示线程的任务函数
def display_thread(buffer):
    while True:
        if buffer.empty():
            continue
        frame = buffer.get()
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# 保存线程的任务函数
def save_thread(output, buffer):
    while True:
        if buffer.empty():
            continue
        frame = buffer.get()
        output.write(frame)
        


if __name__ == '__main__':

    # -----------------------------------------------------------------------------------
    def keyboard_interrupt(signal, frame):
        print("Keyboard Interrupt detected!")
        # sys.exit(0) #用 raise 或者这个皆可
        raise KeyboardInterrupt 

    signal.signal(signal.SIGINT, keyboard_interrupt)

    print("Press Ctrl+C to exit...")

    # 将终端设置为非规范模式,不修改的话需要回车，而且回车也会被读到
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)
    # -----------------------------------------------------------------------------------




    vedio_time_stamp_file = open(vedio_time_stamp_file_path, 'a')

    # 打开USB摄像头
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('N', 'V', '1', '2'))
    capture.set(cv2.CAP_PROP_FPS, 30)
    # capture.set(4, 1080)  # 图片宽度
    # capture.set(3, 1920)  # 图片宽度

    # 检查摄像头是否成功打开
    if not capture.isOpened():
        print("无法打开摄像头")
        exit()

    # 获取摄像头的帧宽度和帧高度
    frame_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 创建视频编写器对象，用于保存视频
    output = cv2.VideoWriter(vedio_file_path, cv2.VideoWriter_fourcc(*'XVID'), 30.0, (frame_width, frame_height))

    loop_time = 1.0/30
    record_start_time = time.time()
    print("开始录制视频")

    try:
        while True:
            

            loop_start_time = time.time()

            ret, frame = capture.read()
            time_stamp = time.time()

            output.write(frame)
            data = f'{time_stamp}\n'
            vedio_time_stamp_file.write(data)
            if int(time_stamp - record_start_time)%10 == 0:
                print(f'vedio recording  time: {time_stamp:.2f}')
            while time.time()-loop_start_time < loop_time :
                pass


    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!")
       

    # 恢复终端设置
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("------恢复终端设置-----")
        # # 关闭视频编写器对象和摄像头
        output.release()
        capture.release()
        vedio_time_stamp_file.close()
        print("record_finished")
        
    

    # # 关闭所有打开的窗口
    # cv2.destroyAllWindows()
    # def keyboard_interrupt(signal, frame):
    #     print("Keyboard Interrupt detected!")
    #     # sys.exit(0) #用 raise 或者这个皆可
    #     raise KeyboardInterrupt 
