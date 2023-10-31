# coding=utf-8

import os.path
import time
import numpy as np
import pygmo.core as pg
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import scipy.io
from spatialmath.base import *
from math3d.transform import Transform as Trans
import rospy
from sensor_msgs.msg import Joy
import cv2 as cv
import math


#高斯分布参数 [       ,               ,               ,           ,   横坐标      ， 纵坐标       ，          ]
fun_left_0 = [0.05410334, 137.06098690, 377.40041913, 431.40073990, 623.18590305, 600.16764162, 0.00402319]
fun_right_0 = [0.07491880, 96.42623581, 99.32096722, 254.80939073, 1143.58609198, 600.94976255, 0.01276680]
fun_left = fun_left_0.copy()
fun_right = fun_right_0.copy()

fun_last_time = 0

k_fun_zoom = 200
k_fun_xy = 100

img_width = 1920
img_height = 1080

img_center_x = img_width /2
img_center_y = img_height /2

img_edge_x = 100
img_edge_y = 50


#手柄切换运镜模式的标识
lap_control_mode = 0  #0:手动运镜，1：自动运镜+手动调节
lock = True
lock_last_button = 0
dx = dy = dz = 0
manual = False
auto_fun = False
auto_angle = True
manual_last_button = 0
auto_fun_last_button = 0
auto_angle_last_button = 0
fun_reset_last_button = False



def joy_fun_adjust(msg):

    global lock
    global lock_last_button
    global dx,dy,dz
    global manual, auto_fun, auto_angle
    global manual_last_button, auto_fun_last_button, auto_angle_last_button, fun_reset_last_button
    global fun_left, fun_right
    global k_fun_xy, k_fun_zoom
    global fun_left_0, fun_right_0
    global fun_last_time

    delta_time = time.time() - fun_last_time
    fun_last_time = time.time()
    print('delta time: ',delta_time)

    if msg.buttons[10]:
        if abs(lock_last_button) < 0.01:
            if lock:
                lock = False
                print('已解锁')
            else:
                lock = True
                print('已锁定')

    lock_last_button = msg.buttons[10]
    if lock:
        dx = dy = dz = 0
        return

    lock_last_button = msg.buttons[10]
    
    threshold = 0.3
    if abs(msg.axes[0]) > threshold:
        dx = msg.axes[0]
    else: 
        dx = 0

    if abs(msg.axes[1]) > threshold:
        dy = msg.axes[1]
    else:
        dy = 0

    if abs(msg.axes[5]) > threshold:
        dz = msg.axes[5]
    else:
        dz = 0

    if dx or dy or dz :
        print(f'joy msg: ({dx:.2f},{dy:.2f},{dz:.2f})')
        if dz:
            left_x = fun_left[4] - img_center_x
            left_y = fun_left[5] - img_center_y
            left_y_x = left_y / left_x
            left_distance = np.sqrt(pow(left_x,2) + pow(left_y,2))
            right_x = fun_right[4] - img_center_x
            right_y = fun_right[5] - img_center_y
            right_y_x = right_y / right_x
            right_distance = np.sqrt(pow(right_x,2) + pow(right_y,2))
            print(f'left : ({left_x}, {left_y})  {left_distance}')
            print(f'right: ({right_x}, {right_y})  {right_distance}')

            left_x = left_x + left_x / (left_distance + right_distance)  * dz * k_fun_zoom * delta_time
            right_x = right_x + right_x  / (left_distance + right_distance) * dz * k_fun_zoom * delta_time
            left_y = left_y_x * left_x
            right_y = right_y_x * right_x
            fun_left[4] = left_x + img_center_x
            fun_left[5] = left_y + img_center_y
            fun_right[4] = right_x + img_center_x
            fun_right[5] = right_y + img_center_y 
            
            


        if dx:
            fun_left[4] = fun_left[4] + k_fun_xy * dx * delta_time
            fun_right[4] = fun_right[4] + k_fun_xy * dx * delta_time

        if dy:
            fun_left[5] = fun_left[5] + k_fun_xy * dy * delta_time
            fun_right[5] = fun_right[5] + k_fun_xy * dy * delta_time


        if fun_left[4] < img_edge_x:
            fun_left[4] = img_edge_x
        if fun_left[5] < img_edge_y:
            fun_left[5] = img_edge_y

        if fun_right[4] < img_edge_x:
            fun_right[4] = img_edge_x
        if fun_right[5] < img_edge_y:
            fun_right[5] = img_edge_y

        if fun_left[4] > img_width-img_edge_x:
            fun_left[4] = img_width-img_edge_x
        if fun_left[5] > img_height-img_edge_y:
            fun_left[5] = img_height-img_edge_y

        if fun_right[4] > img_width-img_edge_x:
            fun_right[4] = img_width-img_edge_x
        if fun_right[5] > img_height-img_edge_y:
            fun_right[5] = img_height-img_edge_y
        
        # print('fun_L :',fun_left)
        # print('fun_R :',fun_right)

        




    
    if (not auto_fun_last_button) and msg.buttons[7]:
        manual = False
        auto_angle = False
        auto_fun = True
        print('Auto_fun')
    if (not auto_angle_last_button) and msg.buttons[9]:
        manual = False
        auto_fun = False
        auto_angle = True
        print('Auto_angle')
    if (not manual_last_button) and msg.buttons[6]:
        auto_fun = False        
        auto_angle = False
        manual = True
        print('Manual')
    
    auto_fun_last_button = msg.buttons[7]
    auto_angle_last_button = msg.buttons[9]
    manual_last_button = msg.buttons[6]
    

    if (not fun_reset_last_button) and msg.buttons[11]:
        fun_left = fun_left_0.copy()
        fun_right = fun_right_0.copy()
        print('fun 复位')
        print('fun_L :',fun_left)
        print('fun_L0 :',fun_left_0)
        print('fun_R :',fun_right)
        print('fun_R0 :',fun_right_0)

    fun_reset_last_button = msg.buttons[11]

    
    

    if lap_control_mode == 0: #0:手动运镜，1：自动运镜+手动调节
        pass

    # print("-------------------------------------")
    # print(msg.axes[0])
    # print(msg.axes[1])
    # print(msg.axes[2])


if __name__ == '__main__':
    rospy.init_node('Optimal', anonymous=True)
    rospy.Subscriber("joy", Joy ,joy_fun_adjust,queue_size=1)
    cv.namedWindow('figure', 0)
    # cv.resizeWindow('figure', 960, 540)

    time.sleep(0.1)
    print('start')
    fun_last_time = time.time()
    while not rospy.is_shutdown():
        img = np.zeros((img_height, img_width, 3), np.uint8)
        img = cv.circle(img,(int(fun_left[4]),int(fun_left[5])), 10, (0,255,0), -1)
        img = cv.circle(img,(int(fun_right[4]),int(fun_right[5])), 10, (255,0,0), -1)
        cv.imshow('figure', img)
        cv.waitKey(1)
        pass
