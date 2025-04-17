'''
基于rokae_ros
选择以下几种机械臂位姿。
注意！！！ 只要在终端按下对应字母或序号，不用回车就会运行。
有两种：
1、进行关节角控制来运行到位姿
2、开启拖动模式，暂时不支持退出拖动模式（rokae_ros的问题）

另外具有记录选定机械臂关节角的功能(注意修改保存路径)

文件路径仅依赖于 lap_set.data_folder

q:记录当前位姿的机器人末端位姿和动捕测到的固连在机器人末端的位姿，以用于两个系统d之间的标定，ctrl c结束程序时，若有数据则进行标定计算

'''
import sys
import os
import numpy as np 
import rospy
import math
from geometry_msgs.msg import PoseStamped
import time
from scipy.linalg import logm
from spatialmath.base import *


sys.path.append(f"{os.path.dirname(__file__)}")
import rokae_basic_fun 

sys.path.append(f"{os.path.dirname(__file__)}/my_tools")
import key_signal

sys.path.append(f"{os.path.dirname(__file__)}/../src/optimal/scripts")
from lap_set_pk import lap_set
#=======================================================================================================================================

joints_record_path = f"{lap_set.data_folder}/pre_data/joints.txt" #用于保存 机械臂-腹腔镜 标定用的关节角
lap_end_list_path = f"{lap_set.data_folder}/pre_data/lap_end_positions.txt" # 用于保存{base}下的 腹腔镜末端 位置（根据几何尺寸推算）
needle_position_path = f"{lap_set.data_folder}/pre_data/needle_position.txt"# 用于保存{base}下的 定位针 位置（根据几何尺寸推算）
lap_shaft_path = f"{lap_set.data_folder}/pre_data/lap_shaft.txt"# 用于保存{base}下的 腹腔镜轴线（根据几何尺寸推算）

T_0_qualisys_path = f'{lap_set.data_folder}/Qualisys_calibration/T_0_cam.txt' #保存 动捕与机器人标定 的结果
T_rob_rigid_path = f'{lap_set.data_folder}/Qualisys_calibration/T_rob_rigid.txt'

T_0_rcm_path = f'{lap_set.data_folder}/coordinate_set/camera_rcm_pose.csv'

joints_record_list = []     #机械臂7个关节角
lap_end_list = []           #根据夹具尺寸计算腹腔镜结构末端位置
lap_shaft_list = []         #前面3位为 points，后面3位 orientation
needle_position_list = []   #根据定位针尺寸计算末端位置

keyboard_monitor = key_signal.keyboard_monitor_class()

joint_velocity = 15/180*math.pi
        

qualisys_rob_CAL_remeasure_times = 10 #每次记录 动捕rigid 和 机器人 位姿以用于两系统标定时，重复采样该times次数， 取平均
rigid_CAL_pose_now = None #用于存放当前收到的用于机器人标定的刚体位姿
rigid_CAL_pose_list = []
rob_CAL_pose_list = []


rospy.init_node('rokae_init_pose')







joints = np.array([
    [0,     43.963,     0,      77.728,     6.349,      59.259,     79.145],    #对准下rcm
    [0,     67.568,     0,      75.524,     5.436,      -52.270,    81.927],    #对准前rcm
    [0,     34.276,     0,      48.969,     5.435,      48.827,     81.927]     #力传感器校准位姿
])
joints = joints * (np.pi/180)


def qualisys_CAL_pub_callback(msg):
    global rigid_CAL_pose_now
    rigid_CAL_pose_now = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                                   msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])
    # print(rigid_CAL_pose_now)

rospy.Subscriber('qualisys_cal_pub', PoseStamped, qualisys_CAL_pub_callback)

def save_data(data_list, file_path, key_word):
        '''
        将 data_list 转为矩阵并存储在 file_path
        并根据 key_word 显示保存数据名称
        Args:
            data_list (list): 需要保存的数据列表 
            file_path (str): 保存路径及文件名
            key_word (str):仅用于程序内显示提示
        '''
        if len(data_list) != 0:
            np.savetxt(file_path, np.array(data_list), delimiter=',')
            print(f'已将 {len(data_list)} 个 {key_word} 储存在 {file_path}')
        else:
            print(f'未记录 {key_word} 信息 {len(data_list)}')

def average_list(data_list):
    return np.mean(np.array(data_list), axis=0).tolist()

def pose_average(list):
    """
    根据输入的pose列表, 求平均
    Args:
        list (n*7_): 各列分别为位置xyz四元数wxyz
    Returns:
        list (7): 平均位置和四元数(已归一化)
    """
    poses_array = np.array(list)
    position = np.mean(poses_array[:,:3], axis=0)
    orientation = np.mean(poses_array[:,3:], axis=0)
    if orientation[0] < 0:
        orientation == -orientation
    orientation = orientation/np.linalg.norm(orientation)
    return position.tolist()+orientation.tolist()
    

def save_all_data():
    if len(needle_position_list) >0:
        needle_position_average = average_list(needle_position_list)
        needle_position_list.append(needle_position_average)
        #下面几行临时添加，用于rcm标定，用完可关掉------------------------------- 
        T_0_rcm = np.loadtxt(T_0_rcm_path, delimiter=',')
        T_0_rcm[:3,3] = np.array(needle_position_average)
        np.savetxt(T_0_rcm_path, T_0_rcm, delimiter=',') 
        #------------------------------- 
    save_data(joints_record_list, joints_record_path, '关节角位置')
    save_data(lap_end_list, lap_end_list_path, '腹腔镜末端位置')
    save_data(needle_position_list, needle_position_path, '标定针位置')
    save_data(lap_shaft_list, lap_shaft_path, "腹腔镜轴线")

def pose_list_to_catted_T(list):
    """
    将n个7dof位姿向量转换为 4*4n的拼合齐次变换矩阵
    Args:
        list (n*7): xyz、wxyz

    Returns:
        T_catted (4*4n)
    """
    length = len(list)
    T_list = []
    for i in range(length):
        T=np.eye(4)
        T[:3,:3] = q2r(list[i][3:],'sxyz')
        T[:3,3] = np.array(list[i][:3])
        T_list.append(T.copy())
    T_catted = np.hstack(T_list)
    return T_catted

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


rokae = rokae_basic_fun.rokae()

rokae.set_mode('jp')

tip_words = f'当前机械臂模式：{rokae.mode()}\n'\
        +'请选择初始化位姿：\n'\
        +'0: 对准下方rcm             \tj: 记录 关节角 \n'\
        +'1: 对准前方rcm             \tl: 记录 腹腔镜 末端位置\n'\
        +'2: 力传感器校准位姿         \to: 记录 腹腔镜 轴线\n'\
        +'d: 拖动模式（珞石自带）      \tn: 记录 定位针 位置\n'\
        +'f: 退出拖动模式（珞石自带）   \tq: 记录当前 动捕与机器人\n'\
        +'z: 绕末端z轴连续转动、\n'\
        +'\n'\
        +'s: 保存所有数据\n'\
        +'c: 保存数据 + 计算腹腔镜轴线交点 (不设置rcm)\n'\
        +'r: 保存数据 + 计算 rcm 点 + 设置 rcm\n'\
        +'ctrl+c: 退出（并保存）\n'

try:
    print(tip_words)
    while not rospy.is_shutdown():
        # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
        rlist, _, _ = keyboard_monitor.detect()
        if rlist:
            # 读取单个字符并处理
            input_data = keyboard_monitor.read_char()
            print(f'\n已选择位姿{input_data}')
            if rokae.mode == 'drag' and input_data!='d':
                print('rokae_init_pose.py: 退出拖动')
                rokae.set_mode('jp')

            if input_data == '0':
                rokae.jp_cmd(joints[0],velocity=joint_velocity)
            elif input_data == '1':
                rokae.jp_cmd(joints[1],velocity=joint_velocity)
            elif input_data == '2':
                rokae.jp_cmd(joints[2],velocity=joint_velocity)
            elif input_data == 'z':
                joint_temp = np.array(rokae.JointState.position).squeeze()
                angle_temp = joint_temp[6]
                for i in range(2):
                    joint_temp[6] = -np.pi
                    rokae.jp_cmd(joint_temp,velocity=joint_velocity*2)
                    joint_temp[6] = np.pi
                    rokae.jp_cmd(joint_temp,velocity=joint_velocity*2)
                joint_temp[6] = angle_temp
                rokae.jp_cmd(joint_temp,velocity=joint_velocity*2)

                
            elif input_data == 'd' and rokae.mode != 'drag':
                rokae.set_mode('drag')
            elif input_data == 'f' and rokae.mode() == 'drag':
                rokae.set_mode('drag_stop')
                print('rokae_init_pose: drag_stop')
                
            elif input_data == 'j':
                joints_record_list.append(np.array(rokae.JointState.position).squeeze())
                print(f'已记录 机械臂 关节角{len(joints_record_list)}：\n{joints_record_list[-1]}')

            elif input_data == 'l':
                lap_end_list.append((rokae.pose.R_matrix()@lap_set.rob_lap_end_vector).squeeze()+rokae.pose.position.array())
                print(f'已记录 腹腔镜末端 位置：{len(lap_end_list)}：\n{lap_end_list[-1]}')
                print(rokae.pose.position)

            elif input_data == 'o':
                shaft_vector = np.zeros(6)
                shaft_vector[:3] = rokae.pose.position.array()
                shaft_vector[3:] = rokae.pose.R_matrix()[:,2].squeeze()
                lap_shaft_list.append(shaft_vector)
                print(f'已记录 腹腔镜 轴线：{len(lap_shaft_list)}：\n{lap_shaft_list[-1]}')
                print(rokae.pose.position)

            elif input_data == 'n':
                needle_position_list.append((rokae.pose.R_matrix()@lap_set.rob_needle_end_vector).squeeze()+rokae.pose.position.array())
                print(f'已记录 定位针 位置：{len(needle_position_list)}：\n{needle_position_list[-1]}')
                print(rokae.pose.position)

            elif input_data == 's':
                save_all_data()
                print(f'已保存数据')

            elif input_data == 'c':
                save_all_data()
                lap_intersection = lap_set.intersection_of_multi_lines(input_file=lap_shaft_path)
                print(f'腹腔镜轴线交点：{lap_intersection}')
                
            elif input_data == 'r':
                save_all_data()
                try:
                    rcm_pose = np.loadtxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv',delimiter=',')
                except:
                    rcm_pose = np.loadtxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv',delimiter=' ')
                lap_intersection = lap_set.intersection_of_multi_lines(input_file=lap_shaft_path)
                print(f'腹腔镜轴线交点：{lap_intersection}')
                rcm_pose[:3,3] = lap_intersection
                print(f'rcm pose:\n{rcm_pose}')
                np.savetxt(f'{lap_set.coordinate_set_folder}/camera_rcm_pose.csv',rcm_pose,delimiter=',')
            
            elif input_data == 'q':
                print(f"开始记录 刚体 及 机器人 位姿\n...")
                time.sleep(0.2)
                temp_rigid_pose_list = []
                temp_rob_pose_list = []
                i = 0
                while rigid_CAL_pose_now is None:
                    print("未接收到刚体信息...")
                    i+=1
                    time.sleep(0.2)
                    if i > 15:
                        rospy.signal_shutdown()
                recorded = False
                for i in range(qualisys_rob_CAL_remeasure_times):
                    if not np.isnan(rigid_CAL_pose_now[0]):
                        recorded = True
                        orientation = rokae.pose.orientation.array('wxyz')
                        if orientation[0]<0:
                            orientation = -orientation
                        temp_rob_pose_list.append([rokae.pose.position.x, rokae.pose.position.y, rokae.pose.position.z, orientation[0], orientation[1], orientation[2], orientation[3]])
                        temp_rigid_pose_list.append(rigid_CAL_pose_now.copy())
                        time.sleep(0.1)
                if recorded:
                    rigid_CAL_pose_list.append(pose_average(temp_rigid_pose_list))
                    rob_CAL_pose_list.append(pose_average(temp_rob_pose_list))
                else:
                    print(f"[WARNING]: 未接收到刚体信息")

                joints_record_list.append(np.array(rokae.JointState.position).squeeze())
                print(f'已记录 机械臂 关节角{len(joints_record_list)}：\n{joints_record_list[-1]}')

                print(f"已记录 刚体 及 机器人 位姿: {len(rigid_CAL_pose_list)}\n----------------------------------------------")
                

                
                    

            # 运动后重新显示提示词
            if input_data != 'j'\
                and input_data != 'l'\
                and input_data != 'n':
                print(f'已抵达位姿{input_data}\n')
                print(tip_words)


        else:
            input_data=''
            # print(f"rokae end:\n{rokae.pose.T_matrix()}")
            # count=0

# 程序中断处理        
except KeyboardInterrupt:
    print("Keyboard Interrupt detected!  (except)")
    # if rokae.mode=='drag':
    #     rokae.set_mode('jp')
    
    
    rospy.signal_shutdown()
    
# 恢复终端设置
finally:
    save_all_data()
    keyboard_monitor.monitor_stop()

    if len(rigid_CAL_pose_list)>0: #计算 动捕 及 机器人 标定关系
        print(f"进行动捕和机器人的标定计算，数据长度: {len(rigid_CAL_pose_list)}")
        assert len(rigid_CAL_pose_list) == len(rob_CAL_pose_list), f"数据长度不等:\nrigid_CAL_pose_list: {len(rigid_CAL_pose_list)}\nrob_CAL_pose_list: {len(rob_CAL_pose_list)}"
        print
        rigid_CAL_T_catted = pose_list_to_catted_T(rigid_CAL_pose_list)
        rob_CAL_T_catted = pose_list_to_catted_T(rob_CAL_pose_list)
        print(f"rigid_CAL_T_catted:\n{rigid_CAL_T_catted}")
        print(f"rob_CAL_T_catted:\n{rob_CAL_T_catted}")
        T_0_qualisys, T_rob_rigid = calibrate_calculate(rob_CAL_T_catted, rigid_CAL_T_catted, T_0_cam_path=T_0_qualisys_path, T_rob_rigid_path=T_rob_rigid_path )
        print(f"计算结果:\nT_0_qualisys:\n{T_0_qualisys}\n\nT_rob_rigid:\n{T_rob_rigid}")
    
    




