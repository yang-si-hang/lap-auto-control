'''
用于眼和机械臂分别固连在h世界的情况，注意眼不在手上。
机械臂末端固连观测目标，B
calibrate_calculate() 用于直接传入测得的机械臂末端变换矩阵和眼观测的目标位姿，解算手眼相对位姿
shiu() 用于解算 AX=XB 方程组

根据 fold_path 中的 joints 自动运动，并采集数据 T_0_rob、T_cam_rigid 保存在fold_path
并计算和保存 T_0_camera、T_rob_rigid 保存在 fold_path

文件路径仅依赖于 lap_set.data_folder
'''

import numpy as np
import time
import time,os

import sys
import rospy
from spatialmath.base import *
import pkg_resources
import xml.etree.ElementTree as ET
import qtm_rt
import asyncio
import math
from scipy.linalg import logm

# sys.path.append("/home/yiliao/wyh/laparoscope_ws/src/optimal/scripts")
sys.path.append(f"{os.path.dirname(__file__)}/../../optimal/scripts")
from lap_set_pk import lap_set

sys.path.append(f"{os.path.dirname(__file__)}/../../../scripts")
import rokae_basic_fun 
#=======================================================================================================

rigid_calibrate = 'calibrate'

step_rigid_get_flag = False


time_start = None

fold_path = f'{lap_set.data_folder}/Qualisys_calibration'
joints_load_path = f'{fold_path}/joints.txt'
T_0_rob_path = f'{fold_path}/T_0_rob.txt'
T_cam_rigid_path = f'{fold_path}/T_cam_rigid.txt'
T_0_qualisys_path = f'{fold_path}/T_0_cam.txt'
T_rob_rigid_path = f'{fold_path}/T_rob_rigid.txt'

# 使用np进行文件保存时，不需要手动打开文件和清空内容

T_cam_rigid = np.identity(4)




# move_joints = np.array([
#     [48.171,    27.197,     -46.464,    88.783,     -7.046,     56.905,     194.308],
#     [25.921,    32.874,     -35.979,    62.769,     13.203,     77.953,     118.375],
#     [60.352,    25.728,     -47.300,    82.807,     -46.655,    72.398,     121.384],
#     [32.295,    41.698,     -33.561,    101.184,    -123.049,   61.561,     158.861],
#     [24.303,    45.684,     -30.820,    86.721,     -129.022,   46.515,     198.211],
#     [41.683,    53.389,     -26.969,    103.943,    -150.339,   59.502,     224.598],
#     [44.573,    46.973,     -30.278,    92.970,     -102.468,   55.449,     184.325],
#     [38.844,    41.885,     -31.808,    70.371,     -31.298,    53.924,     82.236],
#     [22.142,    45.517,     -30.136,    69.967,     -8.609,     33.710,     31.920],
#     [23.762,    41.503,     -32.202,    70.682,     -1.792,     56.846,     17.310],
#     [35.329,    31.845,     -38.786,    74.770,     0.585,      66.592,     -4.109],
#     [38.486,    19.252,     -63.932,    88.609,     38.848,     68.567,     -97.005],
#     [33.681,    53.929,     -25.635,    39.348,     -19.084,    79.543,     11.663],
#     [6.404,     44.173,     -30.565,    67.856,     77.233,     89.220,     -15.740]
# ])
# move_joints = move_joints/180*math.pi

move_joints = np.loadtxt(joints_load_path,delimiter=',')
print(move_joints)

num_of_pose = move_joints.shape[0]
print(f'num of pose: {num_of_pose}')
T_0_rob_catted = np.zeros((4,4*num_of_pose))
T_cam_rigid_catted = np.zeros((4,4*num_of_pose))

T_0_rob_list = []
T_cam_rigid_list = []


QTM_FILE = pkg_resources.resource_filename("qtm_rt", "data/Demo.qtm")


def create_body_index(xml_string):
    """ Extract a name to index dictionary from 6dof settings xml """
    xml = ET.fromstring(xml_string)
    # xml = ET.fromstring(xml_string)

    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index

    return body_to_index

def body_enabled_count(xml_string):
    xml = ET.fromstring(xml_string)
    return sum(enabled.text == "true" for enabled in xml.findall("*/Body/Enabled"))

async def move_and_measure():
    global T_0_rob_list, T_cam_rigid_list, T_0_rob_catted, T_cam_rigid_catted
    global step_rigid_get_flag


    # Connect to qtm
    # connection = await qtm_rt.connect("127.0.0.1")
    connection = await qtm_rt.connect(lap_set.qualisys_master_ip)

    # Connection failed?
    if connection is None:
        print("Failed to connect")
        return

    # Take control of qtm, context manager will automatically release control after scope end
    # async with qtm_rt.TakeControl(connection, "password"):
    async with qtm_rt.TakeControl(connection, lap_set.qualisys_password):

        # realtime = False  #????
        realtime = True

        if realtime:
            # Start new realtime
            await connection.new()
        else:
            # Load qtm file
            await connection.load(QTM_FILE)

            # start rtfromfile
            await connection.start(rtfromfile=True)

    # Get 6dof settings from qtm
    xml_string = await connection.get_parameters(parameters=["6d"])
    body_index = create_body_index(xml_string)

    print("{} of {} 6DoF bodies enabled".format(body_enabled_count(xml_string), len(body_index)))


    def on_packet(packet):
        global step_rigid_get_flag, T_0_rob_catted, T_cam_rigid_catted
        
        info, bodies = packet.get_6d()
        # print(
        #     "Framenumber: {} - Body count: {}".format(
        #         packet.framenumber, info.body_count
        #     )
        # )


        #此处只特定刚体 rigid_calibrate
        rigid_index = body_index[rigid_calibrate]
        position, rotation = bodies[rigid_index]
        
        R = np.array(rotation).reshape(3,3).T #R是反的，是刚体坐标系下相机坐标系的表达,经过此行.T后正常了R是相机坐标系下刚体的位姿
        
        T_cam_rigid[:3, :3] = R
        T_cam_rigid[:3, 3] = np.array(position).squeeze()/1000 #除以1000后单位为米

        if not np.isnan(position[0]):  
            # print(f"{rigid_calibrate} \n{R}\n{position}")
            # print(f"{rigid_calibrate} get")
            step_rigid_get_flag = True
            pass
        
        # else:
            # print(f'can\'t get rigid message!!!')

        # q = r2q(R)
        # print("q: ",q)

        '''数据字符串 xyz、                                                      wxyz、'''
        # data = f'{position[0]/1000},{position[1]/1000},{position[2]/1000},\t{q[0]},{q[1]},{q[2]},{q[3]}\n'
    
        # if not np.isnan(position[0]): print(f'{data}')
        # print(f'{(time.perf_counter()-time_start):.6f} s')
        # print("{} - Pos: {} - Rot: {}".format(calibration_rigid_calibrate, position, rotation))
        

    # Start streaming frames
    await connection.stream_frames(components=["6d"], on_packet=on_packet)
    
    # while not rospy.is_shutdown():
    # while True:
    #     # Wait asynchronously seconds
    #     await asyncio.sleep(1)
    #     pass

    print(f'------- start move -------')
    for i in range(num_of_pose):
        print(f'--- step {i} ---')
        step_rigid_get_flag = False
        rokae.jp_cmd(move_joints[i],velocity=10/180*math.pi)
        await asyncio.sleep(1) #此异步期间会进行on_packet，更新 quealisys 数据
        
        if step_rigid_get_flag:
            T_0_rob_list.append(rokae.pose.T_matrix().copy())
            T_cam_rigid_list.append(T_cam_rigid.copy())
        else:
            print("can't get rigid message!!!")
        

        # T_0_rob_catted[:, 4*i : 4*i+4] = rokae.pose.T_matrix()
        # T_cam_rigid_catted[:, 4*i : 4*i+4] = T_cam_rigid

    print(f'-------- move stop --------')
    
    print(f'length:{len(T_0_rob_list)}')
    T_0_rob_catted = np.hstack(T_0_rob_list)
    T_cam_rigid_catted = np.hstack(T_cam_rigid_list)

    np.savetxt(T_0_rob_path, T_0_rob_catted, delimiter=',')
    np.savetxt(T_cam_rigid_path, T_cam_rigid_catted, delimiter=',')
    print(f'shape of T_0_rob_catted: {T_0_rob_catted.shape}')
    print(f'shape of T_cam_rigid_catted: {T_cam_rigid_catted.shape}')

    # Stop streaming
    await connection.stream_frames_stop()




def calibrate_calculate(_T_0_rob_catted, _T_cam_rigid_catted, T_0_cam_path = None, T_rob_rigid_path = None):
    '''
    用于求解眼在手外，被观测物体在手上的情况
    args:
        _T_0_rob_catted: 4*4n 矩阵 机械臂末端位姿
        _T_cam_rigid_catted: 4*4n 矩阵 固定相机坐标系下机械臂末端固连刚体的位姿
        T_0_cam_path: (str) 如不需要保存最终结果，则None
        T_rob_rigid_path: (str) 如不需要保存最终结果，则None
    return:
        T_0_camera: 4*4 T_0_camera  机械臂基坐标系下camera的位姿
        T_rob_rigid: 机械臂末端坐标系下，机械臂末端固连的被观测物位姿
    '''
    def shiu(A, B):
        # 计算 A 和 B 的维度
        if A.shape != B.shape:
            print(f'A、B 矩阵形状不同：{A.shape} != {B.shape}')
            return
        m, n = A.shape

        if n%4 != 0:
            print(f'矩阵列数({n}),不是4的整倍数')
            return
        n = n // 4


        # 初始化 AA 和 bb
        AA = np.zeros((9*(n-1), 2*n))
        bb = np.zeros((9*(n-1), 1))
        
        # 计算最佳旋转 R
        for i in range(n):
            print(f'i: {i}')
            A1 = logm(A[0:3, 4*i:4*i+3])
            B1 = logm(B[0:3, 4*i:4*i+3])
            a1 = np.array([A1[2, 1], A1[0, 2], A1[1, 0]]).squeeze()
            a1 = a1 / np.linalg.norm(a1)
            b1 = np.array([B1[2, 1], B1[0, 2], B1[1, 0]]).squeeze()
            b1 = b1 / np.linalg.norm(b1)
            v = np.cross(b1, a1)
            w = math.atan2(np.linalg.norm(v), np.dot(b1, a1))  
            v = v / np.linalg.norm(v)
            XP = np.eye(3) * np.cos(w) + np.sin(w) * skew(v) + (1 - np.cos(w)) * v.reshape(3,1) @ v.reshape(1,3)
            Ai, bi = shiu_matrix(a1, XP)
            if i == 0:
                AA[:, :2] = np.tile(-Ai, (n-1, 1))  
                bb[:, 0] = np.tile(-bi.reshape(-1,1), (n-1, 1)).squeeze()
            else:
                AA[9*(i-1):9*i, 2*i:2*i+2] = Ai
                bb[9*(i-1):9*i, 0] = bb[9*(i-1):9*i, 0] + bi.reshape(-1,1).squeeze()
        
        beta = np.linalg.lstsq(AA, bb, rcond=None)[0]
        theta = math.atan2(beta[2*n-1], beta[2*n-2])  # Convert beta components to scalar
        RA = np.eye(3) * np.cos(theta) + np.sin(theta) * skew(a1) +  a1.reshape(3,1) @ a1.reshape(1,3) * (1 - np.cos(theta)) 
        R = RA @ XP
        
        # 计算最佳平移 t
        C = np.zeros((3*n, 3))
        d = np.zeros((3*n, 1))
        I = np.eye(3)
        for i in range(n):
            C[3*i:3*i+3, :] = I - A[0:3, 4*i:4*i+3]
            d[3*i:3*i+3, :] = A[0:3, 4*i+3].reshape(-1,1) - R @ B[0:3, 4*i+3].reshape(-1,1)
        
        t = np.linalg.lstsq(C, d, rcond=None)[0]
        
        # 构建变换矩阵 X
        X = np.block([[R, t], [0, 0, 0, 1]])
        
        return X

    def shiu_matrix(ka1, X):
        ka1 = ka1.squeeze()
        A = np.zeros((9, 2))
        b = np.zeros((9, 1))
        
        A[0, 0] = X[0, 0] - ka1[0] * np.dot(X[:, 0].squeeze(), ka1)
        A[1, 0] = X[0, 1] - ka1[0] * np.dot(X[:, 1].squeeze(), ka1)
        A[2, 0] = X[0, 2] - ka1[0] * np.dot(X[:, 2].squeeze(), ka1)
        A[3, 0] = X[1, 0] - ka1[1] * np.dot(X[:, 0].squeeze(), ka1)
        A[4, 0] = X[1, 1] - ka1[1] * np.dot(X[:, 1].squeeze(), ka1)
        A[5, 0] = X[1, 2] - ka1[1] * np.dot(X[:, 2].squeeze(), ka1)
        A[6, 0] = X[2, 0] - ka1[2] * np.dot(X[:, 0].squeeze(), ka1)
        A[7, 0] = X[2, 1] - ka1[2] * np.dot(X[:, 1].squeeze(), ka1)
        A[8, 0] = X[2, 2] - ka1[2] * np.dot(X[:, 2].squeeze(), ka1)
        
        n = np.cross(X[:, 0], ka1)
        o = np.cross(X[:, 1], ka1)
        a = np.cross(X[:, 2], ka1)
        
        A[0, 1] = -n[0]
        A[1, 1] = -o[0]
        A[2, 1] = -a[0]
        A[3, 1] = -n[1]
        A[4, 1] = -o[1]
        A[5, 1] = -a[1]
        A[6, 1] = -n[2]
        A[7, 1] = -o[2]
        A[8, 1] = -a[2]
        
        n = X[:, 0]
        o = X[:, 1]
        a = X[:, 2]
        
        b[0] = -ka1[0] * np.dot(n, ka1)
        b[1] = -ka1[0] * np.dot(o, ka1)
        b[2] = -ka1[0] * np.dot(a, ka1)
        b[3] = -ka1[1] * np.dot(n, ka1)
        b[4] = -ka1[1] * np.dot(o, ka1)
        b[5] = -ka1[1] * np.dot(a, ka1)
        b[6] = -ka1[2] * np.dot(n, ka1)
        b[7] = -ka1[2] * np.dot(o, ka1)
        b[8] = -ka1[2] * np.dot(a, ka1)
        
        return A, b

    def skew(v):
        return np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])



    if _T_0_rob_catted.shape != _T_cam_rigid_catted.shape:
        print(f'calibrate_calculate  传入矩阵形状不同：{_T_0_rob_catted.shape} != {_T_cam_rigid_catted.shape}')
        return
    # A0,B0 用于求解 T_0_cam ; A1,B1 用于求解 T_rob_rigid
    n = _T_0_rob_catted.shape[1] // 4
    A0 = np.zeros((4,4*(n-1)))
    B0 = np.zeros((4,4*(n-1)))
    A1 = np.zeros((4,4*(n-1)))
    B1 = np.zeros((4,4*(n-1)))
    for i in range(n-1):
        A0[:,4*i:4*i+4] = _T_0_rob_catted[:,4*(i+1):4*(i+1)+4] @ np.linalg.inv(_T_0_rob_catted[:,4*i:4*i+4])
        B0[:,4*i:4*i+4] = _T_cam_rigid_catted[:,4*(i+1):4*(i+1)+4] @ np.linalg.inv(_T_cam_rigid_catted[:,4*i:4*i+4])
        A1[:,4*i:4*i+4] = np.linalg.inv(_T_0_rob_catted[:,4*i:4*i+4]) @ _T_0_rob_catted[:,4*(i+1):4*(i+1)+4]
        B1[:,4*i:4*i+4] = np.linalg.inv(_T_cam_rigid_catted[:,4*i:4*i+4]) @ _T_cam_rigid_catted[:,4*(i+1):4*(i+1)+4]


    T_0_camera = shiu(A0, B0)
    T_rob_rigid = shiu(A1, B1)

    if T_0_cam_path is not None:
        print(f'T_0_camera 保存位置：{T_0_cam_path}')
        np.savetxt(T_0_cam_path, T_0_camera, delimiter=',')
    if T_rob_rigid_path is not None:
        print(f'T_rob_rigid 保存位置：{T_rob_rigid_path}\n')
        np.savetxt(T_rob_rigid_path, T_rob_rigid, delimiter=',')

    return T_0_camera, T_rob_rigid

if __name__ == "__main__":
    time_start = time.perf_counter()
    rospy.init_node('qualisys_rob_calibration', anonymous=True)
    rokae = rokae_basic_fun.rokae()
    time.sleep(1)
    rokae.set_mode('jp')

    

    # Run our asynchronous function until complete
    asyncio.get_event_loop().run_until_complete(move_and_measure())
    print(f'T_0_rob_catted:\n{T_0_rob_catted}')
    T_0_qualisys, T_rob_rigid = calibrate_calculate(T_0_rob_catted, T_cam_rigid_catted, T_0_cam_path=T_0_qualisys_path, T_rob_rigid_path=T_rob_rigid_path )
    print(f'T_0_qualisys:\n{T_0_qualisys}\n')
    print(f'T_rob_rigid:\n{T_rob_rigid}\n')




    
