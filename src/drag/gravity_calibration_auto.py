'''

注意：
    1、正确设置 lap_set.T_rob_sensor
    2、建议运行 rokae_init_pose 
    
    文件路径仅依赖于 lap_set.data_folder
'''

import os.path
import math

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
import time

import sys
from spatialmath.base import trotx, troty, trotz, transl, angvec2tr, rpy2tr
from math3d.transform import Transform as Trans

import random


from spatialmath.base import *
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header




sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src")
import drag.force_sensor_receiver as force_sensor_receiver

sys.path.append(f"{os.path.dirname(__file__)}/../../scripts/my_tools")
import key_signal

sys.path.append(f"{os.path.dirname(__file__)}/../../scripts")
import rokae_basic_fun 

#==============================================================================================

joint_velocity = 25.0/180.0*math.pi
np.set_printoptions(precision=6, suppress=True)


T_rob_sensor = lap_set.T_rob_sensor
R_rob_sensor = T_rob_sensor[:3,:3]


choice = 0



def gravity_compensation(force_file_path, sensor_pose_file_path, G_L_F0_file_path):
    '''
    传入文件路径：
        1、各个测量位置的平均力 n*6
        2、各个测量位置的位姿向量n*6
    return: 
        G:  float   连接刚体所受重力大小 
        L:  list    链接刚体质心位置
        F0: list    测量力0偏
    '''
    force_torque = np.loadtxt(force_file_path, delimiter=',')
    sensor_pose = np.loadtxt(sensor_pose_file_path, delimiter=',')
    points_num = force_torque.shape[0]
    if points_num != sensor_pose.shape[0]:
        print(f'力和姿态数据数量不匹配！')
        return
    force = force_torque[:,:3].reshape(-1,1) #-1 means aoto compute according to the other arguments
    torque = force_torque[:,3:].reshape(-1,1)

    fE_list = []
    for i in range(points_num):
        ff = [[0,                   force[3*i+2][0],    -force[3*i+1][0]],
              [-force[3*i+2][0],    0,                  force[3*i][0]   ],
              [force[3*i+1][0],     -force[3*i][0],     0               ]]
        F_assemble = np.concatenate((ff, np.eye(3)), axis=1)
        fE_list.append(F_assemble)
    fE = np.concatenate(fE_list, axis=0)

    R_assemble_list = []
    for i in range(points_num):
        T_0_sensor = np.array(Trans(sensor_pose[i]).array)
        R_0_sensor = T_0_sensor[:3,:3]
        R_sensor_0 = R_0_sensor.T
        R_assemble = np.concatenate((R_sensor_0, np.eye(3)), axis=1)
        R_assemble_list.append(R_assemble)
    R = np.concatenate(R_assemble_list, axis=0)

    A = np.linalg.solve(R.T @ R, R.T)
    aa = A @ force
    
    print(f'aa:{aa}')
    G = np.array(aa[:3,0]).squeeze()

    force_0 = aa[3:6,0]   #x\y\z


    # ===============
    B = np.linalg.solve(fE.T @ fE, fE.T)
    b = B @ torque


    Lx = b[0][0];Ly = b[1][0];Lz = b[2][0]
    bx = b[3][0];by = b[4][0];bz = b[5][0]
    mx0 = bx - force_0[1] * Lz + force_0[2] * Ly
    my0 = by - force_0[2] * Lx + force_0[0] * Lz
    mz0 = bz - force_0[0] * Ly + force_0[1] * Lx

    L = [Lx,Ly,Lz]
    print(f'L norm:{np.linalg.norm(np.array(L))}')
    F0 = [force_0[0], force_0[1], force_0[2],mx0,my0,mz0]
    G_L_F0 = np.array(G.tolist()+ L + F0).squeeze()
    np.savetxt(G_L_F0_file_path, G_L_F0, delimiter=',')
    return G, L, F0


def move_and_save_data(num_data_point, force_file_path, sensor_pose_file_path, _rokae, F_sensor):
        
        global R_rob_sensor, T_rob_sensor
        R_sensor_rob = R_rob_sensor.T

       

        
        position = _rokae.pose.T_matrix()[:3,3]
        joints_start = list(_rokae.JointState.position)
        # print(f'joints_start: {joints_start}')
        print(f'joints_start: {_rokae.JointState.position}')

            
        F_list = []
        pose_list = []

        for i in range(num_data_point):
            if rospy.is_shutdown():
                return
            print(f'point {i} 采样开始')
            joints = joints_start
            joints[4] = random.uniform(-164.0/180.0*math.pi, 164.0/180.0*math.pi)
            joints[5] = random.uniform(-110.0/180.0*math.pi, 110.0/180.0*math.pi)
            joints[6] = random.uniform(-math.pi, math.pi)
            print(f'目标关节角：\n{np.array(joints)/math.pi*180}')
            _rokae.jp_cmd(joints, velocity=joint_velocity)
            time.sleep(0.5)
            print(f'point {i} 已到达，开始测量...')
            T_0_rob = _rokae.pose.T_matrix()
            T_0_sensor = T_0_rob @ T_rob_sensor
            pose = Trans(T_0_sensor).pose_vector
            pose_list.append(pose)
            F_one_point = []
            for j in range(100):
                 if rospy.is_shutdown():
                     return
                 if j%60 == 0:
                    print(f'{j}th measure: ',np.concatenate((F_sensor.force, F_sensor.torque)))
                 F_one_point.append(np.concatenate((F_sensor.force, F_sensor.torque)))
                 time.sleep(0.01)
            F_temp = np.array(F_one_point)
            # print(F_one_point)
            # print(F_temp)
            F_list.append(F_temp.mean(axis=0))   #按列求平均

        F_matrix = np.array(F_list)
        pose_matrix = np.array(pose_list)

        np.savetxt(force_file_path, F_matrix, delimiter=',')
        np.savetxt(sensor_pose_file_path, pose_list, delimiter=',')





if __name__ == '__main__':
    
    keyboard_monitor = key_signal.keyboard_monitor_class()

    rospy.init_node('calibration', anonymous=True)
    rokae = rokae_basic_fun.rokae()
    rokae.set_mode('jp')

    F_sensor = force_sensor_receiver.force_sensor_receiver_class()


    F_file = f'{lap_set.data_folder}/force_calibration/F.txt'
    pose_file = f'{lap_set.data_folder}/force_calibration/pose.txt'
    G_L_F0_file = f'{lap_set.data_folder}/force_calibration/G_L_F0.txt'

    
    

    try:
        print(f'请输入：\n1.运动并标定\n2.仅显示结果')
        while not rospy.is_shutdown():
            while choice == 0:
                rlist, _, _ = keyboard_monitor.detect()
                if rlist:
                    input_data = keyboard_monitor.read_char()
                    if input_data == '1':
                        choice = 1
                        print('运动至初始位置')
                        rokae.jp_cmd(np.array([0,     34.276,     0,      48.969,     5.435,      48.827,     81.927])* (np.pi/180), velocity=joint_velocity) #
                        # 运动采点，并保存数据
                        move_and_save_data(10, F_file, pose_file, rokae, F_sensor)
                        #根据数据点文件来进行计算
                        G, L, F0 = gravity_compensation(F_file, pose_file, G_L_F0_file)
                        print(f'G:\n{G}')
                        print(f'L:\n{L}')
                        print(f'F0:\n{F0}')

                        F_sensor.set_gravity_args(G, L, F0)
                        break
                    
                    if input_data == '2':
                        choice = 2
                        F_now_pub = rospy.Publisher('gravity_calibrated_force',WrenchStamped, queue_size=1)
                        wrench_stamped_msg = WrenchStamped()
                        break

            T_0_rob = rokae.pose.T_matrix()
            # print(rokae.pose,"\n",T_0_rob)
            T_0_sensor = T_0_rob @ T_rob_sensor
            R_0_sensor = T_0_sensor[:3,:3]
            F_all_now = F_sensor.F
            F_now = F_sensor.pure_force_compute(F_all_now, R_0_sensor)
            print( f'\r=============================\n'\
                    f'G:\t{F_sensor.G}\n'\
                    f'L:\t{F_sensor.M_center}\n'\
                    f'F0:\t{F_sensor.F0}\n'\
                    f'传感器数值: {F_all_now}\n'\
                    f'外力: {F_now}\n'\
                    f'force: {np.linalg.norm(F_now[:3])}\ntorque: {np.linalg.norm(F_now[3:])}')
            time.sleep(0.1) # 粗略延时
            if choice == 2:
                wrench_stamped_msg.header = Header(stamp=rospy.Time.now())
                wrench_stamped_msg.wrench.force.x = F_now[0]
                wrench_stamped_msg.wrench.force.y = F_now[1]
                wrench_stamped_msg.wrench.force.z = F_now[2]

                wrench_stamped_msg.wrench.torque.x = F_now[3]
                wrench_stamped_msg.wrench.torque.y = F_now[4]
                wrench_stamped_msg.wrench.torque.z = F_now[5]

                F_now_pub.publish(wrench_stamped_msg)


     # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!  (except)")
        keyboard_monitor.monitor_stop()
        rospy.signal_shutdown("Shutdown signal received.")
       

    # 恢复终端设置
    finally:
        keyboard_monitor.monitor_stop()
        rospy.signal_shutdown("Shutdown signal received.")

        


         


        

    







