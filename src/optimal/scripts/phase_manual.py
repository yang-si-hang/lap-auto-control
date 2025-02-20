"""
手动切换并发布 phase
"""

import numpy as np
import torch
from torch import tensor
import torch.optim as optim
import time
import matplotlib.pyplot as plt
import os.path
import sys
import rospy
import json
from std_msgs.msg import Float32MultiArray,String
import curses

sys.path.append(f"{os.path.dirname(__file__)}")
from lap_set_pk import lap_set
from tools import *


sys.path.append(f"{os.path.dirname(__file__)}/../../../scripts/my_tools")
import key_signal
import signal



phase_id = 0
phases_list = ["gripping", "needling", "tightening", "knotting", "cutting", "moving", "grabbing", "placing"]
model_ex_phase_list = ["gripping",  #进针打结
                       "needling",
                       "tightening",
                       "knotting",   
                       "gripping", #打结后第一次进针 右到左
                       "needling",
                       "tightening",
                       "gripping", #打结后第二次进针 左到右
                       "needling",
                       "tightening",
                       "gripping", #打结后第三次进针 右到左，并留线环
                       "needling",
                       "tightening",
                       "knotting", #打结
                       "cutting",
                       "moving", #一次取放
                       "grabbing",
                       "moving",
                       "placing",
                       "moving", #二次取放
                       "grabbing",
                       "moving",
                       "placing",
                       "moving"
                       ]
for phase in model_ex_phase_list:
    assert phase in phases_list, f"{phase} not correct"

phase = model_ex_phase_list[phase_id]

phase_pub = rospy.Publisher('/phase',String,queue_size=1)
phase_msg = String()


keyboard_monitor = key_signal.keyboard_monitor_class()


rospy.init_node("phase_manual")
loop_id = 0
try:
    rate = rospy.Rate(50)  
    while not rospy.is_shutdown():
        # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
        rlist, _, _ = keyboard_monitor.detect()
        if rlist:
            # 读取单个字符并处理
            input_data = keyboard_monitor.read_char()
            if input_data == "w" or input_data == "a":
                print(f"last phase")
                phase_id -= 1
                if phase_id <0: phase_id = 0
                phase = model_ex_phase_list[phase_id]
                phase_msg.data = phase
                phase_pub.publish(phase_msg)
                print(f"{phase_id}: {phase}")


            elif input_data == "s" or input_data == "d":
                print(f"next phase")
                phase_id += 1
                if phase_id >= len(model_ex_phase_list):
                    phase_id = len(model_ex_phase_list)-1
                phase = model_ex_phase_list[phase_id]
                phase_msg.data = phase
                phase_pub.publish(phase_msg)
                print(f"{phase_id}: {phase}")
                

        else:
            input_data=''
            # count=0
    
        if loop_id %5 ==0:
            print(f"{phase_id}: {phase}")
            phase_msg.data = phase
            phase_pub.publish(phase_msg)
        loop_id += 1
        rate.sleep()

        
        
# 程序中断处理        
except KeyboardInterrupt:
    print("Keyboard Interrupt detected!  (except)")
    

# 恢复终端设置
finally:
    "finally"
    keyboard_monitor.monitor_stop()