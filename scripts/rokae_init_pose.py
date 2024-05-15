import sys
import os
import numpy as np 
import rospy

sys.path.append(f"{os.path.dirname(__file__)}")
import rokae_basic_fun 


rospy.init_node('rokae_init_pose')


joints = np.array([
    [0,     43.963,     0,      77.728,     6.349,      59.259,     79.145],    #对准下rcm
    [0,     67.568,     0,      75.524,     5.436,      -52.270,    81.927],    #对准前rcm
    [0,     34.276,     0,      48.969,     5.435,      48.827,     81.927]     #力传感器校准位姿
])

joints = joints * (np.pi/180)


rokae = rokae_basic_fun.rokae()

rokae.set_mode('jp')

rokae.jp_cmd(joints[1])



