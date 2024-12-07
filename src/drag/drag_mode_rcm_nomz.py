'''
和 drag_mode_rcm_nomz.py 的不同: # 消除把手偏置力矩

仅由角加速度推算切向加速度

rcm纠偏，以末端z轴作为腹腔镜轴线
运行过程中有多线程按键检测，检测到 'r' 则进行力传感器 F0 清零,并更新校准文件

文件路径仅依赖于 lap_set.data_folder
'''

import os.path
import math

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int8
import time
import logging
import sys
from spatialmath.base import trotx, troty, trotz, transl, angvec2tr, rpy2tr
from math3d.transform import Transform as Trans
from scipy.spatial.transform import Rotation
import math3d as m3d
import random
from geometry_msgs.msg import TwistStamped, Point

np.set_printoptions(precision=6, suppress=True)

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src")
import drag.force_sensor_receiver as force_sensor_receiver

sys.path.append(f"{os.path.dirname(__file__)}/../../scripts/my_tools")
import key_signal

sys.path.append(f"{os.path.dirname(__file__)}/../../scripts")
import rokae_basic_fun 

import threading
#=====================================================================================

__save_data = True
force_sensor_resetting = False

data_fold = f"{lap_set.data_folder}/drag_data"
force_before_filter_path = f'{data_fold}/force_before_filter.txt'
force_filtered_path = f'{data_fold}/force_filtered.txt'
pose_path = f'{data_fold}/pose.txt'
rcm_error_path = f'{data_fold}/rcm_error.txt'
acceleration_linear_path = f'{data_fold}/acceleration_linear.txt'
acceleration_angular_path = f'{data_fold}/acceleration_angular.txt'
velocity_linear_path = f'{data_fold}/velocity_linear.txt'
velocity_angular_path = f'{data_fold}/velocity_angular.txt'
vector_s_rcm_path = f'{data_fold}/vector_s_rcm.txt'
time_calculate_path = f'{data_fold}/time_calculate.txt'
time_pub_path = f'{data_fold}/time_pub.txt'
torque_force_tau_path = f'{data_fold}/torque_force_tau.txt'
torque_damp_linear_tau_path = f'{data_fold}/torque_force_tau.txt'

'''
force_before_filter_file = open(force_before_filter_path, 'a')
force_before_filter_file.truncate(0)
force_filtered_file = open(force_filtered_path, 'a')
force_filtered_file.truncate(0)
pose_file = open(pose_path, 'a')
pose_file.truncate(0)
rcm_error_file = open(rcm_error_path, 'a')
rcm_error_file.truncate(0)
acceleration_linear_file = open(acceleration_linear_path, 'a')
acceleration_linear_file.truncate(0)
acceleration_angular_file = open(acceleration_angular_path, 'a')
acceleration_angular_file.truncate(0)
velocity_linear_file = open(velocity_linear_path, 'a')
velocity_linear_file.truncate(0)
velocity_angular_file = open(velocity_angular_path, 'a')
velocity_angular_file.truncate(0)
vector_s_rcm_file = open(vector_s_rcm_path, 'a')
vector_s_rcm_file.truncate(0)
time_calculate_file = open(time_calculate_path, 'a')
time_calculate_file.truncate(0)
time_pub_file = open(time_pub_path, 'a')
time_pub_file.truncate(0)
torque_force_tau_file = open(torque_force_tau_path, 'a')
torque_force_tau_file.truncate(0)
torque_damp_linear_tau_file = open(torque_damp_linear_tau_path, 'a')
torque_damp_linear_tau_file.truncate(0)
'''

force_before_filter_list = []
force_filtered_list = []
pose_list = []
rcm_error_list = []
acceleration_linear_list = []
acceleration_angular_list = []
velocity_linear_list = []
velocity_angular_list = []
vector_s_rcm_list = []
time_calculate_list = []
time_pub_list = []
torque_force_tau_list = []
torque_damp_linear_tau_list = []


force_threshold = 5
torque_threshold = 0.1

# friction_linear = 1
# friction_angular = 0.05
friction_linear = 0
friction_angular = 0

mass = 0.5
# I_rotation = 0.6
I_rotation_x = 1.2
I_rotation_y = 1.2
I_rotation_z = 0.1
I_matrix = np.zeros((3,3))
I_matrix[0,0] = I_rotation_x
I_matrix[1,1] = I_rotation_y
I_matrix[2,2] = I_rotation_z
I_matrix_inv = np.linalg.inv(I_matrix)

velocity_linear = np.array([0.0, 0.0, 0.0])
velocity_angular = np.array([0.0, 0.0, 0.0])
velocity_linear_norm = 0
velocity_angular_norm = 0
# velocity_linear_rate = 2
# velocity_angular_rate = 1
velocity_linear_limit = 0.15
velocity_angular_limit = 10/180*math.pi


# 200
damping_linear = 150
damping_linear_z = 80
# damping_angular = 0.2
damping_angular_matrix = np.zeros((3,3))
damping_angular_matrix[0,0] = 0.4
damping_angular_matrix[1,1] = 0.4
damping_angular_matrix[2,2] = 0.4

rcm_error_velocity = np.array([0.0, 0.0, 0.0])
rcm_velocity_rate = 1.0/2.5
rcm_error_threshold = 0.005

acceleration_linear = np.array([0.0, 0.0, 0.0])
acceleration_angular = np.array([0.0, 0.0, 0.0])
# acceleration_linear_rate = 0.2
# acceleration_angular_rate = 0.25

keyboard_monitor = key_signal.keyboard_monitor_class()

frequency_calculate = 500
frequency_pub = 50
pub_count = int(frequency_calculate/frequency_pub)
time_step = 1.0/frequency_calculate


T_0_rcm = lap_set.T_0_rcm
rcm_position = T_0_rcm[:3,3].squeeze()

T_rob_sensor = lap_set.T_rob_sensor
R_rob_sensor = T_rob_sensor[:3,:3]

# 各方向上的力分量上限，一旦超过上限，停止运动，以保护力传感器
sensor_safe_force = 300   
sensor_safe_torque = 7.5

def max_norm(__matrix, __axis = 1):
    '''
    求解矩阵中向量的最大模长
    args:
        __matrix:矩阵
        __axis: 0列向量 1行向量
    return: 
        float 最大模长值  
    '''
    # 计算每行向量的模长
    row_norms = np.linalg.norm(__matrix, axis=__axis)
    # 找到模长的最大值
    max_norm = np.max(row_norms)
    return max_norm

def point_to_rcm(__rokae):
    '''
    控制机械臂末端保持原位置，同时指向RCM点
    '''
    T_0_rob = __rokae.pose.T_matrix()
    z_now = T_0_rob[:3,2]
    z_desire = rcm_position - T_0_rob[:3,3]
    z_desire = z_desire/np.linalg.norm(z_desire)
    print(z_desire)
    axis = np.cross(T_0_rob[:3,2],z_desire)
    if np.dot(z_now, z_desire)>=0:
        theta = math.asin(np.linalg.norm(axis))
    else:
        theta = math.pi - math.asin(np.linalg.norm(axis))
        
    axis = axis/np.linalg.norm(axis)
    rotation_matrix = m3d.Orientation(axis * theta).matrix
    transform_matrix = np.eye(4)  # 4x4的单位矩阵
    transform_matrix[:3, :3] = np.array(rotation_matrix)
    T_desir =  transform_matrix @ T_0_rob
    T_desir[:3,3] = T_0_rob[:3,3]
    print(f'rcm position: {rcm_position}')
    print(f'z_desire: {z_desire}')
    print('T_desir','\n',T_desir)
    rokae.cp_cmd(T_desir)
  

def filter_weight_generate(length=20, sigma=10, miu=0):
    '''
    用于生成加权均值滤波的权重
    权重符合高斯分布，权重总和为1，最后一个权重最大
    args:
        length: 权重总个数，即滤波长度
        sigma: >0, 高斯分布的标准差，越大则权重越平均
        miu: <= 0, 高斯分布的均值<=0 则保证最后的权重最大，即最新的测量值占比最高
    return:
        _weights: np.array， 向量。
    '''
    _weights = np.zeros([1,length]).squeeze()
    for i in range(length):
        _weights[-1-i] = math.exp(-pow((float(i)-miu),2)/2.0/pow(sigma,2))
    _weights = _weights /np.sum(_weights)
    # print(f'filter_weight: {_weights}')
    return _weights


def weighted_moving_average_filter(_before_data, _weights, _stemp_num):
    '''
    加权均值滤波,计算量很小，40步6维力信息滤波耗时约 3e-5 s
    args:
        _before_data(list): _stemp_num * _data_length 每个元素为 _data_length 维力信息

    '''
    _data_length = len(_before_data[0])
    _filtered_data  = _weights.reshape([1, _stemp_num]) @ np.array(_before_data).reshape([_stemp_num, _data_length])
    return _filtered_data.squeeze()


def keyboard_listener():
    global force_sensor_resetting
    while True:
        rlist, _, _ = keyboard_monitor.detect()
        if rlist:
            # 读取单个字符并处理
            input_data = keyboard_monitor.read_char()
            print("You typed:", input_data)
            if input_data == 'r':
                force_sensor_resetting = True
                time_temp = time.perf_counter()
                F_sensor.F0_reset(rokae, 2)
                F_sensor.F0_write()
                print(f'time duration of force sensor resseting:{time.perf_counter()-time_temp}')
                force_sensor_resetting = False


# 在后台启动一个监听线程
listener_thread = threading.Thread(target=keyboard_listener)
listener_thread.daemon = True  # 设置为守护线程，主程序退出时自动停止


filter_length = 40 #实时滤波采用的原始数据列表长度
filter_weights = filter_weight_generate(filter_length, 20, 0)
for i in range(filter_length):
    force_before_filter_list.append([0,0,0, 0,0,0])
    force_filtered_list.append([0,0,0, 0,0,0])



if __name__ == "__main__":

    rospy.init_node('drag_mode', anonymous=True)
    rokae = rokae_basic_fun.rokae()
    F_sensor = force_sensor_receiver.force_sensor_receiver_class()
    pub_velocity_cmd = rospy.Publisher('drag_velocity_cmd',TwistStamped,queue_size=1)
    pub_Twist = rospy.Publisher('TwistStamped_test',TwistStamped,queue_size=1)
    # pub_rcm_error = rospy.Publisher('rcm_error',Point,queue_size=1)
    msg_TwistStamped = TwistStamped()
    point_to_rcm(rokae)
    rokae.cp_stop()
    time.sleep(0.5)
    rokae.cv_stop()
    time.sleep(0.5)
    listener_thread.start()


    print(f'======================= start drag ========================')
    time_start = time.perf_counter()
    rate = rospy.Rate(frequency_calculate)
    try:
        count = 0
        while not rospy.is_shutdown():
            if force_sensor_resetting:
                rokae.stop()
                print(f'waiting for force sensor resetting...')
                time.sleep(0.1)
                continue

            # print('==========================')
            T_0_rob = rokae.pose.T_matrix()
            R_0_rob = T_0_rob[:3,:3]
            R_rob_0 = R_0_rob.T

            T_0_sensor = T_0_rob @ T_rob_sensor
            R_0_sensor = T_0_sensor[:3,:3]
            vector_s_rcm = rcm_position - T_0_sensor[:3,3]
            vector_s_rcm_normalized = vector_s_rcm / np.linalg.norm(vector_s_rcm)
            vector_rob_rcm = rcm_position - T_0_rob[:3,3]
            
            
            F_now = F_sensor.pure_force_now(R_0_sensor)
            force_before_filter_list.append(np.array(F_now).squeeze())  #由于滤波需要，无论是否save_data，此列表都要更新
            F_now_filtered = weighted_moving_average_filter(force_before_filter_list[-filter_length: ], filter_weights, filter_length)
            


            force = F_now_filtered[:3]
            torque = F_now_filtered[3:]

            # 消除把手偏置力矩
            l_0_vector = R_0_sensor @ np.array(F_sensor.M_center) #力传感器到把手中心的向量在{0}下的表达
            torque = torque - np.cross(l_0_vector,force)*0.6

            # 检验是否超过传感器安全范围
            if abs(force[0])>sensor_safe_force or abs(force[1])>sensor_safe_force or abs(force[2])>sensor_safe_force or abs(torque[0])>sensor_safe_torque or abs(torque[1])>sensor_safe_torque or abs(torque[2])>sensor_safe_torque:
                rokae.stop()
                print(f'====================\nSensor overload!!!!!\nF_now: {F_now_filtered}\nvelocity:{velocity_linear},{velocity_angular}')
                if __save_data:
                    time_calculate_list.append(time.perf_counter())
                    force_filtered_list.append(np.array(F_now_filtered).squeeze())
                    vector_s_rcm_list.append(np.array([0,0,0]))
                    torque_force_tau_list.append(np.array([0,0,0]))
                    torque_damp_linear_tau_list.append(np.array([0,0,0]))
                velocity_linear = np.array([0,0,0])
                velocity_angular = np.array([0,0,0])
                acceleration_linear = np.array([0,0,0])
                acceleration_linear = np.array([0,0,0])
                continue

            
            # print(f'测量力:{np.linalg.norm(force)}\t测量力矩:{torque}')
            force_norm = np.linalg.norm(force)
            if force_norm < force_threshold:
                force = np.array([0.0, 0.0, 0.0])
            else:
                force = force/force_norm *(force_norm - force_threshold)

            torque_norm = np.linalg.norm(torque)
            if torque_norm < torque_threshold:
                torque = np.array([0.0, 0.0, 0.0])
            else:
                torque = torque/torque_norm * (torque_norm - torque_threshold)

            force_n =np.dot(force, vector_s_rcm_normalized) * vector_s_rcm_normalized
            force_tau = force - force_n
            torque_force_tau = np.cross(-vector_s_rcm, force_tau)  #以 rcm 为轴点的切向力的力矩
            torque = torque + torque_force_tau
            # print(f'切向力法向残余：{np.dot(force_tau,vector_s_rcm_normalized)}')
            # print(f'力：{force} \t力矩:{torque} = {torque-torque_force_tau} + {torque_force_tau}')

            # 阻尼力，以及阻尼力产生的力矩 （在末端坐标系下计算，因为阻尼系数是在末端坐标系，即和腹腔镜绑定）
            velocity_linear_rob = R_rob_0 @ velocity_linear
            force_damp_linear_rob = - np.array([[damping_linear,0,0],[0,damping_linear,0],[0,0,damping_linear_z]]) @ velocity_linear_rob
            force_damp_linear = R_0_rob @ force_damp_linear_rob


            # force_damp_linear =  - damping_linear * velocity_linear
            force_damp_n = np.dot(force_damp_linear, vector_s_rcm_normalized) * vector_s_rcm_normalized
            force_damp_tau = force_damp_linear - force_damp_n
            torque_damp_angular = R_0_rob @ (- damping_angular_matrix @ ( R_rob_0 @ velocity_angular ))
            # torque_damp_angular = -damping_angular * velocity_angular
            torque_damp_linear_tau = np.cross(-vector_s_rcm, force_damp_tau)  #阻尼力切向分量产生的力矩


            
            if np.linalg.norm(velocity_linear) == 0:
                if np.linalg.norm(force) <= friction_linear:
                    acceleration_linear == np.array([0.0, 0.0, 0.0])
                else:
                    acceleration_linear = (force - friction_linear * force/np.linalg.norm(force) )/mass
            else:
                acceleration_linear = (force - friction_linear * velocity_linear/np.linalg.norm(velocity_linear) + force_damp_linear)/mass
            # acceleration_linear_n = np.dot(acceleration_linear, vector_s_rcm_normalized) * vector_s_rcm_normalized


            if np.linalg.norm(velocity_angular) == 0:
                if np.linalg.norm(torque) <= friction_angular:
                    acceleration_angular == np.array([0.0, 0.0, 0.0])
                else:
                    # acceleration_angular= (torque - friction_angular * torque/np.linalg.norm(torque) )/I_rotation
                    acceleration_angular= (R_0_rob @ I_matrix_inv @ R_rob_0 @ (torque - friction_angular * torque/np.linalg.norm(torque) )).squeeze()
            else:
                # acceleration_angular= (torque - friction_angular * velocity_angular/np.linalg.norm(velocity_angular) + torque_damp_angular + torque_damp_linear_tau)/I_rotation
                acceleration_angular= (R_0_rob @ I_matrix_inv @ R_rob_0 @ (torque - friction_angular * velocity_angular/np.linalg.norm(velocity_angular) + torque_damp_angular + torque_damp_linear_tau)).squeeze()
            
            delta_velocity_linear = acceleration_linear * time_step
            delta_velocity_angular = acceleration_angular * time_step
            

            if np.linalg.norm(force) == 0:
                if np.linalg.norm(delta_velocity_linear) > np.linalg.norm(velocity_linear):
                    velocity_linear = np.array([0.0, 0.0, 0.0])
                else:
                    velocity_linear  += delta_velocity_linear
                velocity_linear = np.array([0.0, 0.0, 0.0])
            else: #是否也应该分情况，由于 friction
                velocity_linear  += delta_velocity_linear
            
            if np.linalg.norm(torque) == 0:
                if np.linalg.norm(delta_velocity_angular) > np.linalg.norm(velocity_angular):
                    velocity_angular = np.array([0.0, 0.0, 0.0])
                else:   
                    velocity_angular  += delta_velocity_angular
                velocity_angular = np.array([0.0, 0.0, 0.0])
            else:   
                velocity_angular  += delta_velocity_angular

            # velocity_linear  = velocity_linear  * velocity_linear_rate
            # velocity_angular = velocity_angular * velocity_angular_rate
            # rcm 约束下，只取法向速度
            velocity_linear = np.dot(velocity_linear, vector_s_rcm_normalized) * vector_s_rcm_normalized
            velocity_tau = np.cross(velocity_angular, -vector_rob_rcm)  #在 rcm 约束下，旋转带来的机械臂末端速度
            velocity_linear = velocity_linear + velocity_tau

            velocity_linear_norm = np.linalg.norm(velocity_linear)
            velocity_angular_norm = np.linalg.norm(velocity_angular)
            if velocity_linear_norm > velocity_linear_limit:
                scale_rate = velocity_linear_limit/velocity_linear_norm
                print(f'scale rate: {scale_rate}\nvelocity before scale:{velocity_linear},{velocity_angular}')
                velocity_linear = velocity_linear * scale_rate
                velocity_angular = velocity_angular * scale_rate
                print(f'velocity after scale:{velocity_linear},{velocity_angular}')

            # if velocity_angular_norm > velocity_angular_limit:
            #     velocity_angular = velocity_angular/velocity_angular_norm * velocity_angular_limit

            
            

            # RCM 补偿
            vector_rob_z = T_0_rob[:3,2].squeeze()
            foot_position = T_0_rob[:3,3].squeeze() + vector_rob_z * np.dot(vector_rob_z, vector_rob_rcm)
            rcm_error = foot_position - rcm_position  #由rcm点指向腔镜轴线上的垂足
            if np.linalg.norm(rcm_error) > rcm_error_threshold:
                rcm_error_velocity = - rcm_velocity_rate * rcm_error
            else:
                rcm_error_velocity = np.array([0,0,0])

            
            # print(f'rcm_error: {rcm_error} ({np.linalg.norm(rcm_error)}) \tvelocity: {rcm_error_velocity}')
            # print(  f'加速度:\t{acceleration_linear} \t{acceleration_angular} ')
            # print(  f'速度:\t{velocity_linear} \t{velocity_angular} ')

            # --- 用于观测 rcm 补偿速度（不影响运动，仅用于观测）
            # msg_TwistStamped.header.stamp = rospy.Time.now()
            # msg_TwistStamped.twist.linear.x = rcm_error_velocity[0]
            # msg_TwistStamped.twist.linear.y = rcm_error_velocity[1]
            # msg_TwistStamped.twist.linear.z = rcm_error_velocity[2]
            # pub_Twist.publish(msg_TwistStamped)

            # --- 用于观测 rcm_error（不影响运动，仅用于观测）
            # msg_rcm_error = Point()
            # msg_rcm_error.x = rcm_error[0]
            # msg_rcm_error.y = rcm_error[1]
            # msg_rcm_error.z = rcm_error[2]
            # pub_rcm_error.publish(msg_rcm_error)


            # rlist, _, _ = keyboard_monitor.detect()
            # if rlist:
            #     # 读取单个字符并处理
            #     input_data = keyboard_monitor.read_char()
            #     print("You typed:", input_data)

            #此处每个计算循环存储一次 
            if __save_data:
                # time_calculate_file.write(f'{time.perf_counter()}\n')
                # force_filtered_file.write()

                time_calculate_list.append(time.perf_counter())
                force_filtered_list.append(np.array(F_now_filtered).squeeze())
                vector_s_rcm_list.append(vector_s_rcm.copy())
                torque_force_tau_list.append(torque_force_tau.copy())
                torque_damp_linear_tau_list.append(torque_damp_linear_tau.copy())



            count += 1

            if count >= pub_count:
                count = 0
                # ur的运动指令
                    # rob.my_speedl(velocity_linear.tolist()+[0, 0, 0],0.5,0.2)
                    # rob.my_speedl([0,0,0]+velocity_angular.tolist(),0.5,0.2)
                    # rob.my_speedl(velocity_linear.tolist()+velocity_angular.tolist(),0.5,0.2)
                # 珞石的运动指令
                rokae.cv_cmd((velocity_linear + rcm_error_velocity).tolist()+velocity_angular.tolist() )  # 六自由度运动
                # rokae.cv_cmd(np.array([0,0,0,velocity_angular[0],0,0])) #仅绕x轴转动
                # print(  f'速度1:\t{velocity_linear} \t{velocity_angular} ')
                print(f'力:[{force_norm},{torque_norm}]\t速度:{velocity_linear}|{velocity_angular}')

                # --- 用于观测 速度指令（不影响运动，仅用于观测）
                msg_TwistStamped.header.stamp = rospy.Time.now()
                msg_TwistStamped.twist.linear.x = velocity_linear[0]
                msg_TwistStamped.twist.linear.y = velocity_linear[1]
                msg_TwistStamped.twist.linear.z = velocity_linear[2]
                msg_TwistStamped.twist.angular.x = velocity_angular[0]
                msg_TwistStamped.twist.angular.y = velocity_angular[1]
                msg_TwistStamped.twist.angular.z = velocity_angular[2]
                pub_velocity_cmd.publish(msg_TwistStamped)

                #此处每个发布循环存储一次
                if __save_data:
                    time_pub_list.append(time.perf_counter())
                    pose_list.append(np.concatenate((rokae.pose.position.array(),rokae.pose.orientation.array())).squeeze())
                    rcm_error_list.append(np.linalg.norm(rcm_error))
                    # acceleration_linear_list.append()
                    # acceleration_angular_list = []
                    velocity_linear_list.append(velocity_linear.squeeze().copy())
                    velocity_angular_list.append(velocity_angular.copy())
                    # print(  f'速度3:\t{velocity_linear} \t{velocity_angular} ')

                    # print(len(velocity_angular_list), velocity_angular_list[-5:])
            

            rate.sleep()

    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!  (except)")
       
    # 恢复终端设置
    finally:
        keyboard_monitor.monitor_stop()
        rospy.signal_shutdown("Shutdown signal received.")
        rokae.stop()

        if __save_data:
            print('正在保存数据......')
            # 计算循环中的数据
            np.savetxt(time_calculate_path, np.array(time_calculate_list), delimiter=',')
            np.savetxt(force_before_filter_path, np.array(force_before_filter_list), delimiter=',')
            np.savetxt(force_filtered_path, np.array(force_filtered_list), delimiter=',')
            np.savetxt(vector_s_rcm_path, np.array(vector_s_rcm_list), delimiter=',')
            np.savetxt(torque_force_tau_path, np.array(torque_force_tau_list), delimiter=',')
            np.savetxt(torque_damp_linear_tau_path, np.array(torque_damp_linear_tau_list), delimiter=',')

            # 发布循环中的数据
            np.savetxt(time_pub_path, np.array(time_pub_list), delimiter=',')
            np.savetxt(pose_path, np.array(pose_list), delimiter=',')
            np.savetxt(rcm_error_path, np.array(rcm_error_list), delimiter=',')
            np.savetxt(velocity_angular_path, np.array(velocity_angular_list), delimiter=',')
            np.savetxt(velocity_linear_path, np.array(velocity_linear_list), delimiter=',')
            print(f'数据保存完毕')
            user_input = input("需要计算最大力、速度请输入'1':")
            if user_input == '1':
                print(f'开始计算最大力、速度...')
                force_filtered = np.array(force_filtered_list)
                force = force_filtered[:,:3]
                torque = force_filtered[:,3:6]
                print(f'最大力: {max_norm(force,1)}')
                print(f'最大力矩: {max_norm(torque,1)}')
                print(f'最大线速度: {max_norm(np.array(velocity_linear_list),1)}')
                print(f'最大角速度: {max_norm(np.array(velocity_angular_list),1)}')


