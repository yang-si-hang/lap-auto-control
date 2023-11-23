import urx
import math3d as m3d
import numpy as np
import time

import rtde_receive


import sys
import os


sys.path.append(f'{os.path.dirname(__file__)}/../src/optimal/scripts')
from lap_set_pk import lap_set

import rtde_control

# 在rtde中，读取数据和控制运动是分开的
rtde_c = rtde_control.RTDEControlInterface(lap_set.robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(lap_set.robot_ip)

#----- 常见的读取，也可以读取目标位姿、速度等 ----------------------------------------------
print(f"当前关节角: {rtde_r.getActualQ()}")
print(f"当前TCP位置: {rtde_r.getActualTCPPose()}")
print(f"当前TCP速度: {rtde_r.getTargetTCPSpeed()}")


#----- 末端位置控制 --------------------------------------------------------------------
# rtde_c.moveL([0.3829008765272479, -0.779356486672843, 0.3295970363725074, -2.045600586766512, 2.3108770523885713, -0.36196394363466333], 0.5, 0.3)
# rtde_c.moveL([0.3829008765272479, -0.579356486672843, 0.3295970363725074, -2.045600586766512, 2.3108770523885713, -0.36196394363466333], 0.5, 0.3)



#----- 关节角 -------------------------------------------------------------------------
# joint_q = [[-0.7369511763202112, -1.7268530331053675, -2.125002861022949, -0.6251672071269532, 1.5825108289718628, -0.6084545294391077]]
# rtde_c.moveJ(joint_q)

#----- 关节角速度，2ms一循环，总共2s------------------------------------------------------
# temp_time = time.time()
# for i in range(1000):
#     t_start = rtde_c.initPeriod()
#     joint_speed = [0.0, 0.0, 0.0, 0.0, -0.1, 0.0]
#     rtde_c.speedJ(joint_speed, 0.5, 0.002)              #目前测试情况，频率500hz，无论如何修改，时间都是2ms
#     rtde_c.waitPeriod(t_start)
# print(f'SpeedJ用时：{time.time()-temp_time}')
# rtde_c.speedStop()                                      #速度控制结束一定注意stop


#----- 末端速度控制 ------------------------------------------------------------------
# temp_time = time.time()
# for i in range(1500):
#     t_start = rtde_c.initPeriod()
#     rtde_c.speedL([0, 0, 0.1, 0, 0, 0], 0.1, 0.002)     #目前测试情况，时间要设置在2ms左右，不支持长时间运动，可能会导致程序结束运动不停止，目前测试情况，频率500hz，无论如何修改，时间都是2ms
#     rtde_c.waitPeriod(t_start)
# print(f'SpeedL用时：{time.time()-temp_time}')
# rtde_c.speedStop()                                      #速度控制结束一定注意stop


#----- 运动至接触 ------------------------------------------------------------------
# speed = [0, -0.05, 0, 0, 0, 0]
# rtde_c.moveUntilContact(speed)
# rtde_c.speedStop()


rtde_c.stopScript()



# sleep(0.2)  #leave some time to robot to process the setup commands


# tool=np.array([[0.0],[0.0],[0.08], [1]])
tool=np.array([[0.0],[0.0],[0.06], [1]])

