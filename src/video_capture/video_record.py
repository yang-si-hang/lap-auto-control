'''
输出： recording time 并不是一直输出，而是每十秒进行一段输出，避免终端一直滚动
终止：直接 ctrl c ，捕获到该终端信号后，会进行保存和终端设置。

保存了两种时间戳，注意：单位g不同！！！！！
time.perf_counter() 单位s
capture.get(cv2.CAP_PROP_POS_MSEC)  单位ms
'''


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
import psutil


import signal
import sys

import select

import tty
import termios

sys.path.append(f"{os.path.dirname(__file__)}/../optimal/scripts")
from lap_set_pk import lap_set
# ==================================================================================================
show_flag = False
write_flag = True  # 是否在图像上写字

video_file_path = lap_set.video_file_path
video_time_stamp_file_path = lap_set.video_time_stamp_file_path  # 用于保存视频数据传递进来后，time.perf_counter() 的时间戳，单位s
video_time_stamp_capture_file_path = lap_set.video_time_stamp_cvcap_file_path  # 用于保存 capture.get(cv2.CAP_PROP_POS_MSEC) 时间戳，单位ms


frame_index = 0

def get_system_uptime():
    with open('/proc/uptime', 'r') as f:
        seconds = float(f.read().split()[0])
    return seconds

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




    video_time_stamp_file = open(video_time_stamp_file_path, 'a')
    video_time_stamp_capture_file = open(video_time_stamp_capture_file_path, 'a')
    video_time_stamp_file.truncate(0)
    video_time_stamp_capture_file.truncate(0)

    # 打开USB摄像头
    capture = cv2.VideoCapture(0)
    # capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('N', 'V', '1', '2'))
    # capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('N', 'V', '1', '2'))
    capture.set(cv2.CAP_PROP_FPS, lap_set.video_fps)
    capture.set(4, lap_set.video_height)  # 图片高度
    capture.set(3, lap_set.video_width)  # 图片宽度

    # 设置文本参数
    position = (50, 50)  # 文本起始位置（x, y）
    font = cv2.FONT_HERSHEY_SIMPLEX  # 字体类型
    font_scale = 3  # 字体大小
    color = (255, 255, 255)  # 字体颜色（白色，BGR格式）
    thickness = 3  # 字体厚度

    

    # 检查摄像头是否成功打开
    if not capture.isOpened():
        print("无法打开摄像头")
        exit()

    # 获取摄像头的帧宽度和帧高度
    frame_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 创建视频编写器对象，用于保存视频
    # output = cv2.VideoWriter(video_file_path, cv2.VideoWriter_fourcc(*'XVID'), lap_set.video_fps, (frame_width, frame_height))
    output = cv2.VideoWriter(video_file_path, cv2.VideoWriter_fourcc(*'mp4v'), lap_set.video_fps, (frame_width, frame_height))

    loop_time = 1.0/lap_set.video_fps
    record_start_time = time.perf_counter()
    print("开始录制视频")

    try:
        while True:
            

            loop_start_time = time.perf_counter()

            ret, frame = capture.read()
            if not ret:
                continue
            
            frame_index += 1



            time_stamp_cvcap = capture.get(cv2.CAP_PROP_POS_MSEC) # 单位 ms
            time_stamp = time.perf_counter() # 单位 s

            # print(f'time.time:{time_stamp}')
            # print(f'psutil: {psutil.cpu_times()[3]}')
            # t = os.popen('uptime -p').read()
            # print(f'os:{t}')
            
            
            if not ret:
                continue

            # 在图像上写文字
            if write_flag:
                cv2.putText(frame, f'capture time:{time_stamp_cvcap/1000:.5f}', position, font, font_scale, color, thickness)
                cv2.putText(frame, f'time.perf_counter: {time_stamp:.5f}', (position[0],position[1]+100), font, font_scale, color, thickness)


            # 将颜色通道的顺序改变为RGR
            if lap_set.rgb2bgr:
                frame = frame[:, :, [2, 1, 0]]

            if show_flag:
                cv2.imshow('Frame', frame)
                
            output.write(frame)
            video_time_stamp_file.write(f'{time_stamp}\n')
            video_time_stamp_capture_file.write(f'{time_stamp_cvcap}\n')


            record_time_length = time_stamp - record_start_time
            if int(record_time_length)%10 == 0:
                print(f'video recording  time: {time_stamp:.2f}')
                print(f'录制时长：{int(record_time_length//3600):d}时 {(int(record_time_length)%3600)//60:d}分 {int(record_time_length%60):d}秒')
            
            if show_flag:
                cv2.waitKey(1)  
            
            remaining_time = loop_time - (time.perf_counter() - loop_start_time)-0.001
            if remaining_time > 0:
                time.sleep(remaining_time)
            # while time.perf_counter()-loop_start_time < loop_time :
            #     pass


    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!")
       

    # 恢复终端设置
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("------恢复终端设置-----")
        # # 关闭视频编写器对象和摄像头
        video_time_stamp_file.write(f'{time_stamp}\n')
        video_time_stamp_capture_file.write(f'{time_stamp_cvcap}\n')
        output.release()
        capture.release()
        video_time_stamp_file.close()
        video_time_stamp_capture_file.close()
        print("record_finished")
        print(f'frame_index: {frame_index}')
        
    

    # # 关闭所有打开的窗口
    # cv2.destroyAllWindows()
    # def keyboard_interrupt(signal, frame):
    #     print("Keyboard Interrupt detected!")
    #     # sys.exit(0) #用 raise 或者这个皆可
    #     raise KeyboardInterrupt 
