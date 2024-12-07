'''
力传感器类
创建时会自动订阅ros话题, 自动实时更新force、torque
set_gravity_args: 设置重力平衡相关参数

'''
import os.path
import time
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped
import sys
sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set
import threading
# from RcmControl import GetSpeed, MotionControl
#=======================================================================================================


class force_sensor_receiver_class(object):
    def __init__(self) -> None:
    
        self.force = [0, 0, 0]
        self.torque = [0, 0, 0]
        self.F = [0,0,0,0,0,0]

        self.folder = f'{lap_set.data_folder}/force_calibration'
        temp_array = np.loadtxt(f'{self.folder}/G_L_F0.txt',delimiter=',').squeeze()
        if len(temp_array) != 12:
             raise ValueError("G_L_F0.txt 不合规(应有G、L、F0 3+3+6=12 个元素)")
        self.G = temp_array[0:3] #{0} 下力传感器把手重力
        self.M_center=temp_array[3:6].tolist() #力传感器上把手在力传感器坐标系下的中心位置
        self.F0 = temp_array[6:].tolist() #传感器0偏

        self.R_rob_sensor = lap_set.T_rob_sensor[:3,:3]

        rospy.Subscriber('/Bota_force_sensor/wrenchstamped',WrenchStamped,  self.force_sensor_callback)

        self.lock = threading.Lock()
        
        pass

    def force_sensor_callback(self,msg):
        self.force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        self.torque = [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        self.F = self.force + self.torque

    def print(self):
        print(f'force:\n\tx: {self.force[0]}\n\ty: {self.force[1]}\n\tz: {self.force[2]}')
        print(f'torque:\n\tx: {self.torque[0]}\n\ty: {self.torque[1]}\n\tz: {self.torque[2]}')

    def set_gravity_args(self, G_float, M_center_list, F0_list):
        self.G = G_float
        self.M_center = M_center_list
        self.F0 = F0_list
    
    def pure_force_now(self, R_0_sensor):
        '''
        根据 R_0_sensor 计算 {0} 下的纯外力值(剔除力传感器上把手的重力和重力矩)
        return: 6素素的np.array
        '''
        return self.pure_force_compute(self.force + self.torque, R_0_sensor)

    def pure_force_compute(self, force_torque_list, R_0_sensor):
        F_G_sensor = np.array(force_torque_list) - np.array(self.F0)
        force_G_0 = R_0_sensor @ F_G_sensor[:3]  #含重力的0坐标系下的力
        torque_G_0 = R_0_sensor @ F_G_sensor[3:]
        M_center_vector_0 = R_0_sensor @ np.array(self.M_center) #0坐标系下，sensor中心指向手柄质心
        G_0_vector = np.array(self.G)
        force_0 = force_G_0 - G_0_vector
        torque_0 = torque_G_0 - np.cross(M_center_vector_0, G_0_vector)
        return np.concatenate((force_0, torque_0))
    
    def F0_reset(self, rokae, duration_time ,pure_force_in_theory = np.zeros((1,6)).squeeze()):
        """
        保持静止状态，采集 duration_time 时长数据，并依据当前理论上的外力值，重新计算 F0。
        如果需要写入校准文件，请运行函数 F0_write !!!

        Args:
            rokae: 传入已经创建的 rokae 类对象，便于读取机器人位姿
            duration_time (秒)
            pure_force_in_theory (6维向量): 理论外力(通常默认为0向量)
        """
        with self.lock:
            F0_error_list = []
            F0_error = np.zeros(6)
            loop_time = 0.005
            steps = int(duration_time/loop_time)
            print(f'F0_reset steps:{steps}')
            for i in range(steps):
                R_0_sensor = rokae.pose.R_matrix() @ self.R_rob_sensor
                force = self.pure_force_now(R_0_sensor)
                force_error = force - pure_force_in_theory
                F0_error[:3] = R_0_sensor.T @ force_error[:3]
                F0_error[3:] = R_0_sensor.T @ force_error[3:]
                F0_error_list.append(F0_error.copy())
            
            F0_error = np.mean(np.array(F0_error_list), axis=0)
            F0_new = F0_error + np.array(self.F0)
            self.F0 = F0_new.tolist()
            time.sleep(loop_time)
            print(f'F0 reset: {self.F0}')
    
    def F0_write(self):
        """
            将目前的 self.F0 写入标定文件，通常搭配 F0_reset 使用
        """
        with self.lock:
            array = np.loadtxt(f'{self.folder}/G_L_F0.txt',delimiter=',')
            array[6:] = np.array(self.F0).squeeze()
            np.savetxt(f'{self.folder}/G_L_F0.txt',array,delimiter=',')
            print(f'F0_writed: {self.folder}/G_L_F0.txt')

    def reload_GLF(self):
        with self.lock:
            temp_array = np.loadtxt(f'{self.folder}/G_L_F0.txt',delimiter=',').squeeze()
            if len(temp_array) != 12:
                raise ValueError("G_L_F0.txt 不合规(应有G、L、F0 3+3+6=12 个元素)")
            self.G = temp_array[0:3]
            self.M_center=temp_array[3:6].tolist()
            self.F0 = temp_array[6:].tolist()










# if __name__ == '__main__':
#     rospy.init_node('read_force', anonymous=True)
#     F_sensor = force_sensor_receiver_class()



#     while  not rospy.is_shutdown():
#         F_sensor.print()
#         force_current = np.concatenate((F_sensor.force, F_sensor.torque))
#         print(f'force_current: {force_current}')
#         pass



