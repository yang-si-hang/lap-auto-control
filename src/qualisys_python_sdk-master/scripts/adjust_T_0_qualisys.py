import sys
import os

import numpy as np
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

from spatialmath.base import *
from math3d.transform import Transform as Trans

sys.path.append(f"{os.path.dirname(__file__)}/../../optimal/scripts")
from lap_set_pk import lap_set

def deg_to_rad(degree):
    return degree/180.0*np.pi

fold_path = f'{lap_set.data_folder}/Qualisys_calibration'
T_0_qualisys_path = f'{fold_path}/T_0_cam.txt'
T_0_qualisys_adjusted_path = f'{fold_path}/T_0_cam_adjusted.txt'

T_0_qualisys = np.loadtxt(T_0_qualisys_path, delimiter=',')
#旋转
# adjusted_T_0_qualisys = T_0_qualisys @ troty(deg_to_rad(-9)) @ trotz(deg_to_rad(-6)) @ trotx(deg_to_rad(4))  
adjusted_T_0_qualisys = trotz(deg_to_rad(2)) @ T_0_qualisys @ troty(deg_to_rad(-8.6)) @ trotz(deg_to_rad(-4.3)) @ trotx(deg_to_rad(4.5))  
#平移                                                                   x                                       y                               z
adjusted_T_0_qualisys[:3, 3] = adjusted_T_0_qualisys[:3, 3] +0.118*adjusted_T_0_qualisys[:3, 0] +0.092*adjusted_T_0_qualisys[:3, 1] -0.058*adjusted_T_0_qualisys[:3, 2] #+ np.array([-0.0102, -0.0354, -0.0498])
print(f"origin T_0_qualisys:\n{T_0_qualisys}\nadjusted T_0_qualisys:\n{adjusted_T_0_qualisys}")

np.savetxt(T_0_qualisys_adjusted_path, adjusted_T_0_qualisys, delimiter=',')
