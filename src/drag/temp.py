import math3d as m3d
import math
import numpy as np
axis = [0,0,1]
radian = math.pi/2
rot_matrix = m3d.Orientation([0,0,radian])
print(rot_matrix)



import os

import time

# 设置滴滴声的频率和时长
frequency = 1000  # 频率，单位Hz
duration = 1000  # 时长，单位毫秒
while True:
    # os.system('paplay /usr/share/sounds/freedesktop/stereo/complete.oga')
    os.system('paplay /usr/share/sounds/freedesktop/stereo/suspend-error.oga')
        
