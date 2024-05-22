#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped,TwistStamped
from sensor_msgs.msg import Joy
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy import transpose

class EndEffectorController:
    def __init__(self):
        rospy.init_node('end_effector_controller', anonymous=True)
        # 订阅笛卡尔姿态
        self.cartesian_sub = rospy.Subscriber('/haption/state/pose', PoseStamped, self.cartesian_callback)

        self.velocity_sub=rospy.Subscriber('/haption/state/twist', TwistStamped, self.velocity_callback)
        # 订阅按钮
        self.button_sub = rospy.Subscriber('/haption/state/buttons', Joy, self.button_callback)

        # 发布末端力矩
        self.torque_pub = rospy.Publisher('/haption/command/wrench', WrenchStamped, queue_size=10)

        # 初始化
        self.end_force = WrenchStamped()

        self.position_now=np.array([0.0,0.0,0.0])
        self.position_saved=np.array([0.0,0.0,0.0])
        self.orientation_now=np.array([0.0,0.0,0.0,0.0])
        self.orientation_saved=np.array([0.0,0.0,0.0,0.0])
        self.velocity_now=np.array([0.0,0.0,0.0,0.0,0.0,0.0])

        

        #状态判断，是否刚度保持
        self.is_hold=False
        #设置刚度
        self.stiffness=800.0
        # 设置扭矩刚度
        self.r_stiffness = 0.0
        #设置阻尼
        self.damping=6.0
        self.r_damping=0.04

        
        self.power_button=0
        self.red_button=0

        #设置负载（N）
        self.load=6.0
        

    def cartesian_callback(self, data):
        self.position_now[0]=data.pose.position.x
        self.position_now[1]=data.pose.position.y
        self.position_now[2]=data.pose.position.z
        self.orientation_now[0]=data.pose.orientation.x
        self.orientation_now[1]=data.pose.orientation.y
        self.orientation_now[2]=data.pose.orientation.z
        self.orientation_now[3]=data.pose.orientation.w


    def velocity_callback(self,data):

        self.velocity_now[0]=data.twist.linear.x
        self.velocity_now[1]=data.twist.linear.y
        self.velocity_now[2]=data.twist.linear.z
        self.velocity_now[3]=data.twist.angular.x
        self.velocity_now[4]=data.twist.angular.y
        self.velocity_now[5]=data.twist.angular.z



    def button_callback(self, data):
        self.power_button=data.buttons[0]
        self.red_button=data.buttons[1]

    def control_loop(self):
        rate = rospy.Rate(125)  #500hz
        while not rospy.is_shutdown():
            self.end_force.wrench.force.x=0.0
            self.end_force.wrench.force.y=0.0
            self.end_force.wrench.force.z=0.0
            self.end_force.wrench.torque.x=0.0
            self.end_force.wrench.torque.y=0.0
            self.end_force.wrench.torque.z=0.0
            if self.power_button!=0:
                #刚度力
                
                if self.red_button == 0:
                    if not self.is_hold:
                        self.position_saved=np.copy(self.position_now)
                        self.orientation_saved=np.copy(self.orientation_now)
                        self.is_hold=True
                        # print("flag")
                    force=(self.position_saved-self.position_now)*self.stiffness
                    # print(self.position_saved)
                    # print(self.position_now)

                    r_saved = R.from_quat(self.orientation_saved).as_matrix()
                    r_now = R.from_quat(self.orientation_now).as_matrix()

                    r_diff = np.dot(r_saved,r_now.T)



                    # 计算扭矩
                    rotation_vector = R.from_matrix(r_diff).as_rotvec()

                    print(rotation_vector)
                    torque = rotation_vector * self.r_stiffness

                    self.end_force.header.stamp=rospy.Time.now()
                    self.end_force.wrench.force.x=force[0]
                    self.end_force.wrench.force.y=force[1]
                    self.end_force.wrench.force.z=force[2]
                    self.end_force.wrench.torque.x=torque[0]
                    self.end_force.wrench.torque.y=torque[1]
                    self.end_force.wrench.torque.z=torque[2]
                    # print(force)

                else:
                    self.end_force.header.stamp=rospy.Time.now()
                    self.end_force.wrench.force.x=0.0
                    self.end_force.wrench.force.y=0.0
                    self.end_force.wrench.force.z=0.0
                    self.end_force.wrench.torque.x=0.0
                    self.end_force.wrench.torque.y=0.0
                    self.end_force.wrench.torque.z=0.0
                    self.is_hold=False
                #简单补偿重力
                # self.end_force.wrench.force.z+=self.load
                # #力控下常驻阻尼力
                self.damping_force=np.copy(self.velocity_now)
                self.damping_force[:3]=-self.velocity_now[:3]*self.damping
                self.damping_force[3:]=-self.velocity_now[3:]*self.r_damping
                self.end_force.wrench.force.x+=self.damping_force[0]
                self.end_force.wrench.force.y+=self.damping_force[1]
                self.end_force.wrench.force.z+=self.damping_force[2]
                self.end_force.wrench.torque.x+=self.damping_force[3]
                self.end_force.wrench.torque.y+=self.damping_force[4]
                self.end_force.wrench.torque.z+=self.damping_force[5]
            else:
                self.end_force.header.stamp=rospy.Time.now()
                self.end_force.wrench.force.x=0.0
                self.end_force.wrench.force.y=0.0
                self.end_force.wrench.force.z=0.0
                self.end_force.wrench.torque.x=0.0
                self.end_force.wrench.torque.y=0.0
                self.end_force.wrench.torque.z=0.0
                self.is_hold=False
            
            # 发布末端力矩
            self.torque_pub.publish(self.end_force)
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = EndEffectorController()
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
