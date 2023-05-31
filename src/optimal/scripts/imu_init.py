# coding=utf-8



import sys
import os

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats


sys.path.append("/home/wyh/IMU_drivers/openzen/build")
import openzen
from sensor_msgs.msg import Imu



# imu0 -> instrument_left -----------------------
imu0_quaternionre_read = np.array([0, 0, 0, 1])
q0_base = np.array([0, 0, 0, 1])
q0_init_list = []


#imu1 -> instrument_right -----------------------
imu1_quaternionre_read = np.array([0, 0, 0, 1])
q1_base = np.array([0, 0, 0, 1])
q1_init_list = []


init_num = 200*10

def imu_r_read(msg):
    global imu1_quaternionre_read


    if len(q1_init_list) < init_num:
        imu1_quaternionre_read = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        q1_init_list.append(imu1_quaternionre_read)
        if len(q1_init_list) % 100 == 0:
            print("initing len: ",len(q1_init_list))
    
    pass



if __name__ == '__main__':
    rospy.init_node('init_imu', anonymous=True)
    
    rospy.Subscriber("/imu_r/imu/data",Imu,imu_r_read,queue_size=1)

    print("initing imu: rob base coordination ...")
    while not rospy.is_shutdown() and len(q1_init_list) < init_num:
        pass

    q1_base = np.mean(q1_init_list,axis=0) #与机械臂基座标对齐时，四元数
    

    savefile = f'{os.path.dirname(__file__)}/../data/init_q1_base.csv'
    if not os.path.exists(savefile):
        os.mknod(savefile)
    np.savetxt(savefile, q1_base, delimiter=",")
    print("q1 init finished\nq base: ",q1_base)

    # b = np.loadtxt(savefile, delimiter=",")
    # print(b) 



