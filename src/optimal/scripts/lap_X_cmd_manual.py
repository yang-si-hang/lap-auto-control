"""
手动控制 lap_X_cmd 发布
    alpha   beta    gamma   d
+:  q       w       e       r
-:  a       s       d       f
0:  z       x       c       无
"""


import numpy as np
import os.path
import sys
import rospy
from std_msgs.msg import Float32MultiArray,String


sys.path.append(f"{os.path.dirname(__file__)}")
from lap_set_pk import lap_set
from tools import *


sys.path.append(f"{os.path.dirname(__file__)}/../../../scripts/my_tools")
import key_signal


rospy.init_node('lap_X_cmd_manual')


lap_X_desired_pub = rospy.Publisher('/cmd/lap_X',Float32MultiArray,queue_size=1)
lap_X_desired_msg = Float32MultiArray()
lap_X_desired_msg.data = [0, 0, 0, 0.16]
lap_X_desired_msg.data[0] = -10/180 * np.pi
lap_X_desired_msg.data[1] = 0/180 * np.pi
lap_X_desired_msg.data[2] = 0
lap_X_desired_msg.data[3] = 0.16

step_length = [1/180*np.pi, 1/180*np.pi, 1/180*np.pi, 0.005]

keyboard_monitor = key_signal.keyboard_monitor_class()



loop_id = 0
try:
    print(f"lap_X_cmd: {lap_X_desired_msg.data}")
    rate = rospy.Rate(100)  
    while not rospy.is_shutdown():
        # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
        rlist, _, _ = keyboard_monitor.detect()
        if rlist:
            # 读取单个字符并处理
            input_data = keyboard_monitor.read_char()
            if input_data == "q" :
                lap_X_desired_msg.data[0] += step_length[0]
            elif input_data == "a":
                lap_X_desired_msg.data[0] -= step_length[0]
            elif input_data == "z":
                lap_X_desired_msg.data[0] = 0
            elif input_data == "w" :
                lap_X_desired_msg.data[1] += step_length[1]
            elif input_data == "s":
                lap_X_desired_msg.data[1] -= step_length[1]
            elif input_data == "x":
                lap_X_desired_msg.data[1] = 0
            elif input_data == "e" :
                lap_X_desired_msg.data[2] += step_length[2]
            elif input_data == "d":
                lap_X_desired_msg.data[2] -= step_length[2]
            elif input_data == "c":
                lap_X_desired_msg.data[2] = 0
            elif input_data == "r" :
                lap_X_desired_msg.data[3] += step_length[3]
            elif input_data == "f":
                lap_X_desired_msg.data[3] -= step_length[3]
            print(f"input: {input_data} || lap_X_cmd: {np.array(lap_X_desired_msg.data[:3])/np.pi*180}, {lap_X_desired_msg.data[3]}")


        else:
            if loop_id % 100 == 0:
                print(f"(+ -): alpha:(q a) beta:(w s) gamma:(e d) d:(r f)")
            # count=0
    
        if loop_id %2 ==0:
            lap_X_desired_pub.publish(lap_X_desired_msg)
            
        loop_id += 1
        rate.sleep()

        
        
# 程序中断处理        
except KeyboardInterrupt:
    print("Keyboard Interrupt detected!  (except)")
    

# 恢复终端设置
finally:
    "finally"
    keyboard_monitor.monitor_stop()

