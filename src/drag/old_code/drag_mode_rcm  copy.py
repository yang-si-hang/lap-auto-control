'''
仅由角加速度推算切向加速度

rcm纠偏，以末端z轴作为腹腔镜轴线
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


__save_data = True

data_fold = f"{os.path.dirname(__file__)}/data/"
force_before_filter_path = f'{data_fold}force_before_filter.txt'
force_filtered_path = f'{data_fold}force_filtered.txt'
pose_path = f'{data_fold}pose.txt'
rcm_error_path = f'{data_fold}rcm_error.txt'
acceleration_linear_path = f'{data_fold}acceleration_linear.txt'
acceleration_angular_path = f'{data_fold}acceleration_angular.txt'
velocity_linear_path = f'{data_fold}velocity_linear.txt'
velocity_angular_path = f'{data_fold}velocity_angular.txt'

force_before_filter_list = []
force_filtered_list = []
pose_list = []
rcm_error_list = []
acceleration_linear_list = []
acceleration_angular_list = []
velocity_linear_list = []
velocity_angular_list = []


force_threshold = 5
torque_threshold = 0.2

friction_linear = 1
friction_angular = 0.05

mass = 0.5
# I_rotation = 0.6
I_rotation_x = 0.6
I_rotation_y = 0.6
I_rotation_z = 0.05
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
velocity_linear_limit = 0.1
velocity_angular_limit = 10/180*math.pi


# 200
damping_linear = 200
damping_linear_z = 100
# damping_angular = 0.2
damping_angular_matrix = np.zeros((3,3))
damping_angular_matrix[0,0] = 0.2
damping_angular_matrix[1,1] = 0.2
damping_angular_matrix[2,2] = 0.2

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




def point_to_rcm(__rokae):
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


def weighted_moving_average_filter(_before_data, _weights, _length):
    '''
    加权均值滤波
    args:
        _before_data(list): filter_length * 6 每个元素为6维力信息

    '''
    _filtered_data  = _weights.reshape([1, _length]) @ np.array(_before_data).reshape([_length, 6])
    return _filtered_data



filter_length = 20 #实时滤波采用的原始数据列表长度
filter_weights = filter_weight_generate(filter_length, 10, 0)
for i in range(filter_length):
    force_before_filter_list.append([0,0,0, 0,0,0])
    force_filtered_list.append([0,0,0, 0,0,0])



if __name__ == "__main__":

    rospy.init_node('drag_mode', anonymous=True)
    rokae = rokae_basic_fun.rokae()
    F_sensor = force_sensor_receiver.force_sensor_receiver_class()
    pub_Twist = rospy.Publisher('TwistStamped_test',TwistStamped,queue_size=1)
    pub_rcm_error = rospy.Publisher('rcm_error',Point,queue_size=1)
    msg_TwistStamped = TwistStamped()
    point_to_rcm(rokae)
    rokae.cp_stop()
    time.sleep(0.5)
    rokae.cv_stop()
    time.sleep(0.5)

    print(f'======================= start drag ========================')
    rate = rospy.Rate(frequency_calculate)
    try:
        count = 0
        while not rospy.is_shutdown():
            # print('==========================')
            T_0_rob = rokae.pose.T_matrix()
            T_0_sensor = T_0_rob @ T_rob_sensor
            R_0_sensor = T_0_sensor[:3,:3]
            vector_s_rcm = rcm_position - T_0_sensor[:3,3]
            vector_s_rcm_normalized = vector_s_rcm / np.linalg.norm(vector_s_rcm)
            vector_rob_rcm = rcm_position - T_0_rob[:3,3]
            
            F_now = F_sensor.pure_force_now(R_0_sensor)
            force_before_filter_list.append(np.array(F_now).squeeze())
            force = F_now[:3]
            torque = F_now[3:]
            F_now = weighted_moving_average_filter(force_before_filter_list[-filter_length: ], filter_weights, filter_length)
            force_filtered_list.append(np.array(F_now).squeeze())

            
            # print(f'测量力:{np.linalg.norm(force)}\t测量力矩:{torque}')
            if np.linalg.norm(force) < force_threshold:
                force = np.array([0.0, 0.0, 0.0])
            # else:
            #     print(f'force {np.linalg.norm(force)} > force_threshold {force_threshold}!!!!!!!!')
            if np.linalg.norm(torque) < torque_threshold:
                torque = np.array([0.0, 0.0, 0.0])
            force_n =np.dot(force, vector_s_rcm_normalized) * vector_s_rcm_normalized
            force_tau = force - force_n
            torque_force_tau = np.cross(-vector_s_rcm, force_tau)  #以 rcm 为轴点的切向力的力矩
            torque = torque + torque_force_tau
            # print(f'切向力法向残余：{np.dot(force_tau,vector_s_rcm_normalized)}')
            # print(f'力：{force} \t力矩:{torque} = {torque-torque_force_tau} + {torque_force_tau}')

            # 阻尼力，以及阻尼力产生的力矩
            velocity_linear_rob = T_0_rob[:3,:3].T @ velocity_linear
            force_damp_linear_rob = - np.array([[damping_linear,0,0],[0,damping_linear,0],[0,0,damping_linear_z]]) @ velocity_linear_rob
            force_damp_linear = T_0_rob[:3,:3] @ force_damp_linear_rob


            # force_damp_linear =  - damping_linear * velocity_linear
            force_damp_n = np.dot(force_damp_linear, vector_s_rcm_normalized) * vector_s_rcm_normalized
            force_damp_tau = force_damp_linear - force_damp_n
            torque_damp_angular = T_0_rob[:3,:3] @ (- damping_angular_matrix @ ( T_0_rob[:3,:3].T @ velocity_angular ))
            # torque_damp_angular = -damping_angular * velocity_angular
            torque_damp_linear_tau = np.cross(-vector_s_rcm, force_damp_tau)


            
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
                    acceleration_angular= (I_matrix_inv @ (torque - friction_angular * torque/np.linalg.norm(torque) )).squeeze()
            else:
                # acceleration_angular= (torque - friction_angular * velocity_angular/np.linalg.norm(velocity_angular) + torque_damp_angular + torque_damp_linear_tau)/I_rotation
                acceleration_angular= (I_matrix_inv @ (torque - friction_angular * velocity_angular/np.linalg.norm(velocity_angular) + torque_damp_angular + torque_damp_linear_tau)).squeeze()
            
            delta_velocity_linear = acceleration_linear * time_step
            delta_velocity_angular = acceleration_angular * time_step
            

            if np.linalg.norm(force) == 0:
                if np.linalg.norm(delta_velocity_linear) > np.linalg.norm(velocity_linear):
                    velocity_linear = np.array([0.0, 0.0, 0.0])
                else:
                    velocity_linear  += delta_velocity_linear
                velocity_linear = np.array([0.0, 0.0, 0.0])
            else:
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

            velocity_linear_norm = np.linalg.norm(velocity_linear)
            velocity_angular_norm = np.linalg.norm(velocity_angular)
            if velocity_linear_norm > velocity_linear_limit:
                scale_rate = velocity_linear_limit/velocity_linear_norm
                velocity_linear = velocity_linear * scale_rate
                velocity_angular = velocity_angular * scale_rate

            # if velocity_angular_norm > velocity_angular_limit:
            #     velocity_angular = velocity_angular/velocity_angular_norm * velocity_angular_limit

            velocity_tau = np.cross(velocity_angular, -vector_rob_rcm)  #在 rcm 约束下，旋转带来的机械臂末端速度
            velocity_linear = velocity_linear + velocity_tau
            
            vector_rob_z = T_0_rob[:3,2].squeeze()
            foot_position = T_0_rob[:3,3].squeeze() + vector_rob_z * np.dot(vector_rob_z, vector_rob_rcm)
            rcm_error = foot_position - rcm_position  #由rcm点指向腔镜轴线上的垂足

            if np.linalg.norm(rcm_error) > rcm_error_threshold:
                rcm_error_velocity = - rcm_velocity_rate * rcm_error
            else:
                rcm_error_velocity = np.array([0,0,0])

            
            print(f'rcm_error: {rcm_error} ({np.linalg.norm(rcm_error)}) \tvelocity: {rcm_error_velocity}')



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


            count += 1
            if count >= pub_count:
                count = 0
                # rob.my_speedl(velocity_linear.tolist()+[0, 0, 0],0.5,0.2)
                # rob.my_speedl([0,0,0]+velocity_angular.tolist(),0.5,0.2)
                # rob.my_speedl(velocity_linear.tolist()+velocity_angular.tolist(),0.5,0.2)
                rokae.cv_cmd((velocity_linear + rcm_error_velocity).tolist()+velocity_angular.tolist() )

                if __save_data:
                    pose_list.append(np.concatenate((rokae.pose.position.array(),rokae.pose.orientation.array())).squeeze())
                    rcm_error_list.append(np.linalg.norm(rcm_error))
                    # acceleration_linear_list.append()
                    # acceleration_angular_list = []
                    velocity_linear_list.append(velocity_linear.squeeze())
                    velocity_angular_list.append(velocity_angular.squeeze())
                    

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
            np.savetxt(pose_path, np.array(pose_list), delimiter=',')
            np.savetxt(force_before_filter_path, np.array(force_before_filter_list), delimiter=',')
            np.savetxt(force_filtered_path, np.array(force_filtered_list), delimiter=',')
            np.savetxt(rcm_error_path, np.array(rcm_error_list), delimiter=',')
            np.savetxt(velocity_angular_path, np.array(velocity_angular_list), delimiter=',')
            np.savetxt(velocity_linear_path, np.array(velocity_linear_list), delimiter=',')



