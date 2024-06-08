import numpy as np
import time,os
import sys
import rospy
from spatialmath.base import *
from scipy.linalg import logm
from geometry_msgs.msg import PoseStamped
import random


haption_pose = PoseStamped()
rospy.init_node('pub_test', anonymous=True)
pub = rospy.Publisher('/haption2/state/pose',PoseStamped,queue_size=1)

i=0
pub_frequency = 100
rate = rospy.Rate(pub_frequency)
while not rospy.is_shutdown():
    haption_pose.header.stamp = rospy.Time.now()
    haption_pose.pose.position.x = random.uniform(-1.0,1)
    haption_pose.pose.position.y = random.uniform(-1.0,1)
    haption_pose.pose.position.z = random.uniform(-1.0,1)
    temp_array = np.random.uniform(-1.0, 1.0, (1,4)).squeeze()
    temp_array = temp_array/np.linalg.norm(temp_array)
    haption_pose.pose.orientation.w = temp_array[0]
    haption_pose.pose.orientation.x = temp_array[1]
    haption_pose.pose.orientation.y = temp_array[2]
    haption_pose.pose.orientation.z = temp_array[3]
    if i % pub_frequency == 0:
        print(f'publishing...\ntime stamp:{haption_pose.header.stamp}\n{[haption_pose.pose.position.x, haption_pose.pose.position.y, haption_pose.pose.position.z]}\n{[haption_pose.pose.orientation.w, haption_pose.pose.orientation.x, haption_pose.pose.orientation.y, haption_pose.pose.orientation.z]}')
    pub.publish(haption_pose)
    rate.sleep()
