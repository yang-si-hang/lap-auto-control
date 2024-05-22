#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
import numpy as np
from scipy.spatial.transform import Rotation as R

class EndEffectorController:
    def __init__(self):
        rospy.init_node('end_effector_controller', anonymous=True)
        # 订阅笛卡尔姿态
        self.cartesian_sub = rospy.Subscriber('/haption/state/pose', PoseStamped, self.cartesian_callback)

        # 发布末端力矩
        self.torque_pub = rospy.Publisher('/haption/command/pose1', PoseStamped, queue_size=1)

        self.pose_now=PoseStamped()

        self.pose_pub=PoseStamped()

        self.is_init=False

        

    def cartesian_callback(self, data):
        self.pose_now=data

    def button_callback(self, data):
        self.power_button=data.buttons[0]
        self.red_button=data.buttons[1]

    def control_loop(self):
        hz=125
        rate = rospy.Rate(hz)  
        duration=10.0
        length=0.15
        time=0.0
        while not rospy.is_shutdown():
            if not self.is_init:
                self.pose_pub=self.pose_now
                self.is_init=True
            self.pose_pub.header.stamp=rospy.Time.now()
            self.pose_pub.pose.position.x+=length/(duration*hz)
            time+=1.0/hz
            if time>duration:
                break
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = EndEffectorController()
        controller.control_loop()
    except rospy.ROSInterruptException:
        pass
