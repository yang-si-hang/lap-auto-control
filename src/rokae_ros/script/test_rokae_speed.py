from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy
import time
import numpy as np
import math
import math3d as m3d
from scipy.spatial.transform import Rotation 
from spatialmath.base import *
import random

pose_now = []

def state_callback(msg):
    global pose_now
    pose_now = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]
    print(f'timestamp:\t{msg.header.stamp}\npose: \n{msg.pose.position.x}\norientation:\n{msg.pose.orientation.w}')

def move_circle(pub):
    global pose_now

    radius = 0.05 
    theta = 0
    num_step = 10*1000
    center = np.array([pose_now[0] + radius, pose_now[1], pose_now[2]])
    rate = rospy.Rate(1000)
    for i in range(num_step+1):
        theta = i/num_step * 2* math.pi
        desire_position = [center[0] - radius*np.cos(theta), center[1]-np.sin(theta)*radius, center[2]]
        msg_pub = PoseStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.pose.position.x = desire_position[0]
        msg_pub.pose.position.y = desire_position[1]
        msg_pub.pose.position.z = desire_position[2]
        msg_pub.pose.orientation.w = pose_now[3]
        msg_pub.pose.orientation.x = pose_now[4]
        msg_pub.pose.orientation.y = pose_now[5]
        msg_pub.pose.orientation.z = pose_now[6]
        pub.publish(msg_pub)
        print(f'publish {i}/{num_step}')
        rate.sleep()

def speed_stop(pub):

    frequency = 1000
    rate = rospy.Rate(frequency)
    for j in range(10):
        msg_pub = TwistStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.twist.linear.x = 0
        msg_pub.twist.linear.y = 0
        msg_pub.twist.linear.z = 0

        msg_pub.twist.angular.x = 0
        msg_pub.twist.angular.y = 0
        msg_pub.twist.angular.z = 0

        pub.publish(msg_pub)
        rate.sleep()

def speed_random(pub):
    
    frequency = 1000
    time_step = 1.0/frequency
    continue_time = 2
    num_step = 3
    v_linear_min = 0.03
    v_linear_max = 0.05
    v_angular_min = 0.08
    v_angular_max = 0.12

    speed_stop(pub)

    for i in range(num_step):

        v_linear = np.random.rand(3)
        # v_linear = v_linear.squeeze()
        v_linear_norm = np.linalg.norm(v_linear)
        v_linear_normalized = v_linear / v_linear_norm
        v_linear = v_linear_normalized * random.uniform(v_linear_min , v_linear_max) * random.choice([-1,1])

        v_angular = np.random.rand(3)
        v_angular_norm = np.linalg.norm(v_angular)
        v_angular_normalized = v_angular / v_angular_norm
        v_angular = v_angular_normalized * random.uniform(v_angular_min , v_angular_max) * random.choice([-1,1])
        # v_angular = v_angular.squeeze()

        rate = rospy.Rate(frequency)
        for j in range(continue_time * frequency):
            msg_pub = TwistStamped()
            msg_pub.header.stamp = rospy.Time.now()
            msg_pub.twist.linear.x = v_linear[0]
            msg_pub.twist.linear.y = v_linear[1]
            msg_pub.twist.linear.z = v_linear[2]

            msg_pub.twist.linear.x = 0
            msg_pub.twist.linear.y = 0
            msg_pub.twist.linear.z = 0

            msg_pub.twist.angular.x = v_angular[0]
            msg_pub.twist.angular.y = v_angular[1]
            msg_pub.twist.angular.z = v_angular[2]

            # msg_pub.twist.angular.x = 0
            # msg_pub.twist.angular.y = 0
            # msg_pub.twist.angular.z = 0

            pub.publish(msg_pub)
            rate.sleep()

    speed_stop(pub)

def speed_cirle_time(pub):
    '''
    速度仅与时间步骤有关，与当前位置无关
    '''
    global pose_now


    frequency = 1000
    time_step = 1.0/frequency
    radius = 0.05 
    theta = 0
    num_step = 10*frequency
    center_circle = np.array([pose_now[0] + radius, pose_now[1], pose_now[2]])
    center_cone = center_circle - np.array([0, 0, radius*math.sqrt(3)])
    rate = rospy.Rate(frequency)
    for i in range(num_step+1):
        theta = i/num_step * 2* math.pi
    
        velocity_linear = np.array([math.sin(theta), -math.cos(theta), 0 ]) * 0.1

  
        
        msg_pub = TwistStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.twist.linear.x = velocity_linear[0]
        msg_pub.twist.linear.y = velocity_linear[1]
        msg_pub.twist.linear.z = velocity_linear[2]
        msg_pub.twist.angular.x = 0
        msg_pub.twist.angular.y = 0
        msg_pub.twist.angular.z = 0
       
        pub.publish(msg_pub)
        
        rate.sleep()


def speed_circle(pub):
    global pose_now

    frequency = 1000
    time_step = 1.0/frequency
    radius = 0.05 
    theta = 0
    num_step = 10*frequency
    center_circle = np.array([pose_now[0] + radius, pose_now[1], pose_now[2]])
    center_cone = center_circle - np.array([0, 0, radius*math.sqrt(3)])
    rate = rospy.Rate(frequency)
    for i in range(num_step+1):
        theta = i/num_step * 2* math.pi
        desire_position = np.array([center_circle[0] - radius*np.cos(theta), center_circle[1]-np.sin(theta)*radius, center_circle[2]])
        cur_position = np.array(pose_now[0:3])

        desire_coor_z = center_cone - desire_position
        desire_coor_z = desire_coor_z / np.linalg.norm(desire_coor_z)

        desire_coor_y = np.cross(desire_coor_z, center_circle - desire_position)
        desire_coor_y = desire_coor_y / np.linalg.norm(desire_coor_y)

        desire_coor_x = np.cross(desire_coor_y, desire_coor_z)
        desire_coor_x = desire_coor_x / np.linalg.norm(desire_coor_x)

        desire_R = np.column_stack((desire_coor_x, desire_coor_y, desire_coor_z ))

        cur_R = q2r(pose_now[3:7])
        # cur_coor_x = cur_R[:,0]
        # cur_coor_y = cur_R[:,1]
        # cur_coor_z = cur_R[:,2]

        R_rotation = desire_R @ cur_R.T
        r = Rotation.from_matrix(R_rotation)
        rot_axis = r.as_rotvec() #模长为角度
        # rot_angle = np.linalg.norm(rot_axis)
        # rot_axis = rot_axis/rot_angle

        velocity_angular = rot_axis / time_step
        velocity_linear = (desire_position-cur_position)/time_step

        velocity_angular_norm = np.linalg.norm(velocity_angular)
        velocity_linear_norm = np.linalg.norm(velocity_linear)

        if velocity_linear_norm > 0.2:
            velocity_linear = velocity_linear/velocity_linear_norm * 0.2

        if velocity_angular_norm > math.pi/6 :
            velocity_angular = velocity_angular/velocity_angular_norm * math.pi/6

        
        
        msg_pub = TwistStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.twist.linear.x = velocity_linear[0]
        msg_pub.twist.linear.y = velocity_linear[1]
        msg_pub.twist.linear.z = velocity_linear[2]
        msg_pub.twist.angular.x = velocity_angular[0]
        msg_pub.twist.angular.y = velocity_angular[1]
        msg_pub.twist.angular.z = velocity_angular[2]
       
        pub.publish(msg_pub)
        # print(f'publish {i}/{num_step}')
        if i % 100 == 0:
            print(f'线速度 {np.linalg.norm(velocity_linear)} {velocity_linear} || 角速度 {np.linalg.norm(velocity_angular)} {velocity_angular}')
        rate.sleep()

def speed_stop(pub):
    global pose_now

    frequency = 1000
    rate = rospy.Rate(frequency)
    for i in range(frequency):
        msg_pub = TwistStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.twist.linear.x = 0
        msg_pub.twist.linear.y = 0
        msg_pub.twist.linear.z = 0
        msg_pub.twist.angular.x = 0
        msg_pub.twist.angular.y = 0
        msg_pub.twist.angular.z = 0

        pub.publish(msg_pub)
        rate.sleep()


def speed_line(pub):
    global pose_now

    frequency = 1000
    rate = rospy.Rate(frequency)
    start_time = time.time()
    continue_time = 3
    while time.time() < start_time+continue_time:
        msg_pub = TwistStamped()
        msg_pub.header.stamp = rospy.Time.now()
        msg_pub.twist.linear.x = 0
        msg_pub.twist.linear.y = 0.05
        msg_pub.twist.linear.z = 0
        msg_pub.twist.angular.x = 0
        msg_pub.twist.angular.y = 0
        msg_pub.twist.angular.z = 0

        pub.publish(msg_pub)
        rate.sleep()

    # start_time = time.time()
    # while time.time() < start_time+continue_time:
    #     msg_pub = TwistStamped()
    #     msg_pub.header.stamp = rospy.Time.now()
    #     msg_pub.twist.linear.x = 0
    #     msg_pub.twist.linear.y = 0
    #     msg_pub.twist.linear.z = 0
    #     msg_pub.twist.angular.x = 0
    #     msg_pub.twist.angular.y = 0
    #     msg_pub.twist.angular.z = 0

    #     pub.publish(msg_pub)
    #     rate.sleep()
    
    
    


if __name__ == "__main__":

    rospy.init_node('rokae_test', anonymous=True)
    rospy.Subscriber('/rokae/state/CartesianPose', PoseStamped, state_callback)
    pose_pub = rospy.Publisher('/rokae/command/CartesianPose',PoseStamped,queue_size=1)
    speed_pub = rospy.Publisher('/rokae/command/Twist',TwistStamped,queue_size=1)
    time.sleep(0.5)

    # move_circle(pose_pub)
    # speed_circle(speed_pub)
    # speed_line(speed_pub)
    speed_random(speed_pub)
    # speed_cirle_time(speed_pub)

    print(f'stopping')
    speed_stop(speed_pub)
    print(f'stopped')

    # while not rospy.is_shutdown():
    #     time.sleep(0.1)
