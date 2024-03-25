
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

    # 打开USB摄像头
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('N', 'V', '1', '2'))
    capture.set(cv2.CAP_PROP_FPS, 30)
    capture.set(4, 1080)  # 图片宽度
    capture.set(3, 1920)  # 图片宽度

    # 检查摄像头是否成功打开
    if not capture.isOpened():
        print("无法打开摄像头")
        exit()

    # 获取摄像头的帧宽度和帧高度
    frame_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 创建视频编写器对象，用于保存视频
    output = cv2.VideoWriter('/home/yiliao/870evo_1t/Experiment_Data/20230629/output_video.avi', cv2.VideoWriter_fourcc(*'XVID'), 30.0, (frame_width, frame_height))

    # 创建缓冲区队列
    buffer = queue.Queue(maxsize=200)
    

    # i=0
    # while i<3600:
    #     ret, frame = capture.read()
    #     output.write(frame)
    #     i += 1
        


    # 创建捕获线程
    capture_t = threading.Thread(target=capture_thread, args=(capture, buffer))
    capture_t.start()

    # 创建显示线程
    display_t = threading.Thread(target=display_thread, args=(buffer,))
    display_t.start()

    # 创建保存线程
    save_t = threading.Thread(target=save_thread, args=(output, buffer,))
    save_t.start()

    # 等待捕获线程完成
    capture_t.join()

    # 等待显示线程完成
    display_t.join()

    # 等待保存线程完成
    save_t.join()

    # # 关闭视频编写器对象和摄像头
    output.release()
    capture.release()

    # # 关闭所有打开的窗口
    # cv2.destroyAllWindows()
    # def keyboard_interrupt(signal, frame):
    #     print("Keyboard Interrupt detected!")
    #     # sys.exit(0) #用 raise 或者这个皆可
    #     raise KeyboardInterrupt 
