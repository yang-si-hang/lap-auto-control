import math3d as m3d
import math
import numpy as np
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R


axis = [0,0,1]
radian = math.pi/2
rot_matrix = m3d.Orientation([0,0,radian])
print(rot_matrix)



import os

import time



class mine_pose:
    def __init__(self):
        self.position = self.position_class()
        self.orientation = self.orientation_class()

    class position_class:
        def __init__(self):
            self.x = 0
            self.y = 0
            self.z = 0

        def __str__(self):
            return f"Position:  (x= {self.x:.4f}, \ty= {self.y:.4f}, \tz= {self.z:.4f})"

    class orientation_class:
        def __init__(self):
            self.w = 0
            self.x = 0
            self.y = 0
            self.z = 0
        def __str__(self):
            return f"Orientation:  (w= {self.w:.4f}, \tx= {self.x:.4f}, \ty= {self.y:.4f}, \tz= {self.z:.4f})"
    
    def __str__(self):
        return f"Pose:\n{self.position}\n{self.orientation}"




pose_now = mine_pose()
print(pose_now)
# # 设置滴滴声的频率和时长
# frequency = 1000  # 频率，单位Hz
# duration = 1000  # 时长，单位毫秒
# while True:
#     # os.system('paplay /usr/share/sounds/freedesktop/stereo/complete.oga')
#     os.system('paplay /usr/share/sounds/freedesktop/stereo/suspend-error.oga')
        
j= JointState()
print(j)
print(j.position)


from spatialmath.base import *

x=0.04215641630189087
y = 0.9882752646719122
z = -0.10224511017349353
w = -0.1052652613536581
r1 = R.from_quat([x,y,z,w])
r2 = q2r([w,x,y,z])


print(f'r1:\n{r1.as_matrix()}')
print(f'r2:\n{np.array(r2) - np.array([[-0.974284 , 0.061799 ,-0.216683], [ 0.10485  , 0.975538 ,-0.193217], [ 0.199442 ,-0.210968 ,-0.95693  ]])}')
print(f'{r1.as_quat()}')
