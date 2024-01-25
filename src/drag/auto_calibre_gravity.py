import os.path

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int8
import urx
import time
import logging
import sys
from spatialmath.base import trotx, troty, trotz, transl, angvec2tr, rpy2tr
from math3d.transform import Transform as Trans
np.set_printoptions(precision=6, suppress=True)

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src")
import drag.force_sensor_receiver as force_sensor_receiver

joints = np.zeros((6,6))
force_record = np.zeros((6,6))
force_read_times = 400
# force_read_times = 4

def position_init():
    global joints
    joints[0] = [ 3.28403377532959, -1.3797438780414026, -1.9157269636737269, -1.4168666044818323, 1.570819616317749, 4.854894161224365]
    joints[1] = [3.284177780151367, -1.4196918646441858, -2.053347412739889, -1.2394312063800257, -1.5708192030536097, 4.569805145263672]
    joints[2] = [3.28403377532959, -1.3796723524676722, -1.9157031218158167, 0.1539444923400879, 1.5708799362182617, 4.854894161224365]
    joints[3] = [3.28403377532959, -1.3796842733966272, -1.9156907240497034, 0.1539444923400879, 1.5708439350128174, 1.7133067846298218]
    joints[4] = [3.2840218544006348, -1.3797314802752894, -1.9156668821917933, 0.1539684534072876, 1.5708439350128174, 3.284222364425659]
    joints[5] = [3.2839980125427246, -1.3797081152545374, -1.9156549612628382, 0.15381228923797607, 1.5708439350128174, 0.14248232543468475]




if __name__ == '__main__':
    rospy.init_node('auto_calibre_gravity', anonymous=True)

    # rob = urx.Robot(lap_set.robot_ip)
    # rob.set_tcp((0,0,0,0,0,0)) #暂时设置为0，到达初始位姿后，会重设 tcp
    # rob.set_payload(0.5, (0,0,0))
    F_sensor = force_sensor_receiver.force_sensor_receiver_class()
    position_init()
    print(f'position init joints:\n{joints}')

    for point_index in range(6):
        # rob.movej(joints[point_index])
        # rob._wait_for_move()

        print(f'point {point_index} measuring...')
        for i in range(force_read_times):  #在每个姿态下都重复 force_read_times 次数的力信息采集
            force_current = np.concatenate((F_sensor.force, F_sensor.torque))
            force_record[point_index] += force_current
            time.sleep(0.005)

    force_record = force_record / force_read_times

    print(f'force_record:\n{force_record}\nmatrix size:{force_record.shape}')
    # print(np.concatenate((force_record, np.eye(6)), axis=1)) #尝试矩阵拼接
    # rob.close()


        


