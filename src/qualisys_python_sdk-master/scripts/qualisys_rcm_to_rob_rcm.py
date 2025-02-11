import numpy as np
import time
import time,os

import sys
# from spatialmath.base import *
from scipy.linalg import logm

# sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
sys.path.append(f"{os.path.dirname(__file__)}/../../optimal/scripts")
from lap_set_pk import lap_set

sys.path.append(f"{os.path.dirname(__file__)}/../../../scripts")
import rokae_basic_fun 
#=======================================================================================================

rigid_calibrate = lap_set.rigid_names['qualisys_rob_calibration']
assert rigid_calibrate is not None, f'未正确设置刚体名称'


step_rigid_get_flag = False


time_start = None

fold_path = f'{lap_set.data_folder}/Qualisys_calibration'
T_0_qualisys_path = f'{fold_path}/T_0_cam.txt'

T_0_qualisys = np.loadtxt(T_0_qualisys_path,delimiter=',')
qualisys_rcm = np.loadtxt(f'{lap_set.coordinate_set_folder}/qualisys_rcm.txt', delimiter=',')
rob0_rcm = T_0_qualisys @ qualisys_rcm

try:
    rcm_pose = np.loadtxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv',delimiter=',')
except:
    rcm_pose = np.loadtxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv',delimiter=' ')

print(f'old rcm:\n{rcm_pose}\nnew rcm:\n{rob0_rcm}')


# np.savetxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv', rob0_rcm, delimiter=',')
