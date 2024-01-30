import os.path
import math

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
from scipy.spatial.transform import Rotation
import math3d as m3d
import random
from geometry_msgs.msg import TwistStamped

np.set_printoptions(precision=6, suppress=True)

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src/optimal/scripts")
from lap_set_pk import lap_set

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/src")
import drag.force_sensor_receiver as force_sensor_receiver

sys.path.append(f"{os.path.dirname(__file__)}/../../scripts/my_tools")
import key_signal

force_threshold = 5
torque_threshold = 0.2

friction_linear = 2
friction_angular = 0.05

mass = 1
I_rotation = 0.3025

velocity_linear = np.array([0.0, 0.0, 0.0])
velocity_angular = np.array([0.0, 0.0, 0.0])
velocity_linear_norm = 0
velocity_angular_norm = 0
velocity_linear_rate = 2
velocity_angular_rate = 1
velocity_linear_limit = 0.1
velocity_angular_limit = 30/180*math.pi

damping_linear = 2550
damping_angular = 1.0

acceleration_linear = np.array([0.0, 0.0, 0.0])
acceleration_angular = np.array([0.0, 0.0, 0.0])
acceleration_linear_rate = 0.1
acceleration_angular_rate = 1

keyboard_monitor = key_signal.keyboard_monitor_class()

frequency = 500
frequency_pub = 50
pub_count = int(frequency/frequency_pub)
time_step = 1.0/frequency


R_rob_sensor = np.array([   [0,    1,  0   ],
                            [0,    0,  -1  ],
                            [-1,   0,  0   ]])

# 暂时只在T中设置了R
T_rob_sensor = np.array([   [-1,   0,  0,   0],
                            [0,    0,  -1,  0],
                            [0,   -1,  0,   0],
                            [0,    0,  0,   1]])


if __name__ == "__main__":

    rospy.init_node('drag_mode', anonymous=True)
    rob = urx.Robot(lap_set.robot_ip)
    F_sensor = force_sensor_receiver.force_sensor_receiver_class()
    pub_Twist = rospy.Publisher('TwistStamped_test',TwistStamped,queue_size=1)
    msg_TwistStamped = TwistStamped()
    time.sleep(0.5)

    rate = rospy.Rate(frequency)
    try:
        count = 0
        while not rospy.is_shutdown():
            T_0_sensor = rob.get_pose().array @ T_rob_sensor
            R_0_sensor = T_0_sensor[:3,:3]
            F_now = F_sensor.pure_force_now(R_0_sensor)
            force = F_now[:3]
            torque = F_now[3:]
            print(f'力：{force} \t力矩:{torque}')

            if np.linalg.norm(force) < force_threshold:
                force = np.array([0.0, 0.0, 0.0])
            else:
                print(f'force {np.linalg.norm(force)} > force_threshold {force_threshold}!!!!!!!!')
            if np.linalg.norm(torque) < torque_threshold:
                torque = np.array([0.0, 0.0, 0.0])

            if np.linalg.norm(velocity_linear) == 0:
                if np.linalg.norm(force) <= friction_linear:
                    acceleration_linear == np.array([0.0, 0.0, 0.0])
                else:
                    acceleration_linear = (force - friction_linear * force/np.linalg.norm(force) )/mass * acceleration_linear_rate
            else:
                acceleration_linear = (force - friction_linear * velocity_linear/np.linalg.norm(velocity_linear) - damping_linear * velocity_linear)/mass * acceleration_linear_rate


            if np.linalg.norm(velocity_angular) == 0:
                if np.linalg.norm(torque) <= friction_angular:
                    acceleration_angular == np.array([0.0, 0.0, 0.0])
                else:
                    acceleration_angular= (torque - friction_angular * torque/np.linalg.norm(torque) )/I_rotation * acceleration_angular_rate
            else:
                acceleration_angular= (torque - friction_angular * velocity_angular/np.linalg.norm(velocity_angular) - damping_angular * velocity_angular)/I_rotation * acceleration_angular_rate
            
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

            velocity_linear  = velocity_linear  * velocity_linear_rate
            velocity_angular = velocity_angular * velocity_angular_rate

            velocity_linear_norm = np.linalg.norm(velocity_linear)
            velocity_angular_norm = np.linalg.norm(velocity_angular)
            if velocity_linear_norm > velocity_linear_limit:
                velocity_linear = velocity_linear/velocity_linear_norm * velocity_linear_limit
            if velocity_angular_norm > velocity_angular_limit:
                velocity_angular = velocity_angular/velocity_angular_norm * velocity_angular_limit  
            

            print(  f'加速度:\t{acceleration_linear} \t{acceleration_angular} ')
            print(  f'速度:\t{velocity_linear} \t{velocity_angular} ')

            msg_TwistStamped.header.stamp = rospy.Time.now()
            msg_TwistStamped.twist.linear.x = velocity_linear[0]
            msg_TwistStamped.twist.linear.y = velocity_linear[1]
            msg_TwistStamped.twist.linear.z = velocity_linear[2]
            msg_TwistStamped.twist.angular.x = velocity_angular[0]
            msg_TwistStamped.twist.angular.x = velocity_angular[1]
            msg_TwistStamped.twist.angular.x = velocity_angular[2]
            count += 1
            if count >= pub_count:
                count = 0
                # rob.my_speedl(velocity_linear.tolist()+[0, 0, 0],0.5,0.2)
                # rob.my_speedl([0,0,0]+velocity_angular.tolist(),0.5,0.2)
                rob.my_speedl(velocity_linear.tolist()+velocity_angular.tolist(),0.5,0.2)
                pub_Twist.publish(msg_TwistStamped)



            rate.sleep()

    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!  (except)")
       
    # 恢复终端设置
    finally:
        keyboard_monitor.monitor_stop()
        rospy.signal_shutdown("Shutdown signal received.")
        rob.close()


