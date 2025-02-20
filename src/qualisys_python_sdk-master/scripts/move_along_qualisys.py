"""
根据T_0_qualisys 控制机械臂在qualisys坐标系运动
以便检查校准
"""
import os, sys
import numpy as np
import torch
from torch import tensor
import torch.optim as optim
import time
import matplotlib.pyplot as plt
import os.path
import sys
import rospy
from std_msgs.msg import Float32MultiArray, String
import math
import json
from collections import deque

sys.path.append(f"{os.path.dirname(__file__)}/../../optimal/scripts")
from lap_set_pk import lap_set
sys.path.append("/home/irobotcare/wyh/laparoscope_ws/scripts")
import rokae_basic_fun 


T_0_qualisys = lap_set.T_0_qualisys
print(f"T_0_qualisys dot:{np.dot(T_0_qualisys[:3,0], T_0_qualisys[:3,1])}")
print(f"T_0_qualisys dot:{np.dot(T_0_qualisys[:3,2], T_0_qualisys[:3,1])}")
print(f"T_0_qualisys dot:{np.dot(T_0_qualisys[:3,0], T_0_qualisys[:3,2])}")

qualisys_rigid_end_dict = {}


def qualisys_rigid_end_json_callback(msg):
    global qualisys_rigid_end_dict
    qualisys_rigid_end_dict = json.loads(msg.data)



rospy.init_node("move_along_qualisys")
rokae = rokae_basic_fun.rokae()

rospy.Subscriber('/qualisys_rigid_end_json',String, qualisys_rigid_end_json_callback)

T_qualisys_0 = np.linalg.inv(T_0_qualisys)
T_0_rob_start = rokae.pose.T_matrix()
T_qualisys_rob_start = T_qualisys_0 @ T_0_rob_start
T_qualisys_x0 = T_qualisys_rob_start.copy()
T_qualisys_x0[0,3] = 0
T_0_rob_qualisysx0 = T_0_qualisys @ T_qualisys_x0

T_0_rob_up = T_0_rob_start.copy()
T_0_rob_up[:3,3] = T_0_rob_start[:3,3] + 0.15 * T_0_qualisys[:3,2]/np.linalg.norm(T_0_qualisys[:3,2])

T_0_rob_down = T_0_rob_start.copy()
T_0_rob_down[:3,3] = T_0_rob_start[:3,3] - 0.15 * T_0_qualisys[:3,2]/np.linalg.norm(T_0_qualisys[:3,2])

T_0_rob_forward = T_0_rob_start.copy()
T_0_rob_forward[:3,3] = T_0_rob_start[:3,3] + 0.15 * T_0_qualisys[:3,1]/np.linalg.norm(T_0_qualisys[:3,1])

T_0_rob_left = T_0_rob_start.copy()
T_0_rob_left[:3,3] = T_0_rob_start[:3,3] - 0.15 * T_0_qualisys[:3,0]/np.linalg.norm(T_0_qualisys[:3,0])
# print(f"start:\n{T_0_rob_start}\nup:\n{T_0_rob_up}\ndown:\n{T_0_rob_down}")
# exit()
def rigid_end_mean(rigid_name, loop_num = 20, time_step = 0.01):
    """
    测量刚体末端位置，取平均
    Args:
        rigid_name 
        loop_num: 测量次数
        time_step : 每次测量时间间隔
    """
    global qualisys_rigid_end_dict
    rigid_p = []
    for i in range(loop_num):
        if not np.isnan(qualisys_rigid_end_dict[rigid_name][0]):
            rigid_p.append(qualisys_rigid_end_dict[rigid_name])
        time.sleep(time_step)
    rigid_p_array = np.array(rigid_p)
    average_p = rigid_p_array.mean(axis=0)
    return average_p



def move(T, linear_speed=0.01, angular_speed=2/180*math.pi, rigid_name = None):
    global qualisys_rigid_end_dict
    if rigid_name is not None:
        rigid_p_start = rigid_end_mean(rigid_name, 20, 0.01)
    
    rokae.cp_cmd(T, velocity_linear = linear_speed, velocity_angular=angular_speed, PRINT=False)

    if rigid_name is not None:
        rigid_p_end = rigid_end_mean(rigid_name, 20, 0.01)
        rigid_p_step = rigid_p_end - rigid_p_start
        print(f"moved to {T[:3,3]}, rigid_p_step: {rigid_p_step}")


        
while not 'lap' in qualisys_rigid_end_dict:
    pass


exit()
# while not rospy.is_shutdown():
print(f"forward")
move(T_0_rob_forward.copy(), linear_speed=0.02, rigid_name='lap')
time.sleep(1)
move(T_0_rob_start.copy(), linear_speed=0.03, rigid_name='lap')
time.sleep(1)

print(f"up")
move(T_0_rob_up.copy(), linear_speed=0.02, rigid_name='lap')
time.sleep(1)
move(T_0_rob_start.copy(), linear_speed=0.03, rigid_name='lap')
time.sleep(1)

print(f"left")
move(T_0_rob_left.copy(), linear_speed=0.02, rigid_name='lap')
time.sleep(1)
move(T_0_rob_start.copy(), linear_speed=0.03, rigid_name='lap')
time.sleep(1)