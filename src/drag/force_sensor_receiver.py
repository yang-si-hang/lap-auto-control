import os.path
import time
import numpy as np
import rospy
from geometry_msgs.msg import WrenchStamped


# from RcmControl import GetSpeed, MotionControl
'''
力传感器类
创建时会自动订阅ros话题, 自动实时更新force、torque
set_gravity_args: 设置重力平衡相关参数

'''
class force_sensor_receiver_class(object):
    def __init__(self) -> None:
    
        self.force = [0, 0, 0]
        self.torque = [0, 0, 0]
        self.F = [0,0,0,0,0,0]
    
        self.G = 0
        self.M_center=[0, 0, 0]
        self.F0 = [0, 0, 0, 0, 0, 0]

        rospy.Subscriber('/bus0/ft_sensor0/ft_sensor_readings/wrench',WrenchStamped,  self.force_sensor_callback)
        
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
        return self.pure_force_compute(self.force + self.torque, R_0_sensor)

    def pure_force_compute(self, force_torque_list, R_0_sensor):
        F_G_sensor = np.array(force_torque_list) - np.array(self.F0)
        force_G_0 = R_0_sensor @ F_G_sensor[:3]  #含重力的0坐标系下的力
        torque_G_0 = R_0_sensor @ F_G_sensor[3:]
        M_center_vector_0 = R_0_sensor @ np.array(self.M_center) #0坐标系下，sensor中心指向手柄质心
        G_0_vector = np.array([0, 0, -self.G])
        force_0 = force_G_0 - G_0_vector
        torque_0 = torque_G_0 - np.cross(M_center_vector_0, G_0_vector)
        return np.concatenate((force_0, torque_0))









# if __name__ == '__main__':
#     rospy.init_node('read_force', anonymous=True)
#     F_sensor = force_sensor_receiver_class()



#     while  not rospy.is_shutdown():
#         F_sensor.print()
#         force_current = np.concatenate((F_sensor.force, F_sensor.torque))
#         print(f'force_current: {force_current}')
#         pass



