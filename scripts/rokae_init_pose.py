'''
基于rokae_ros
选择以下几种机械臂位姿。
注意！！！ 只要在终端按下对应字母或序号，不用回车就会运行。
有两种：
1、进行关节角控制来运行到位姿
2、开启拖动模式，暂时不支持退出拖动模式（rokae_ros的问题）

另外具有记录选定机械臂关节角的功能(注意修改保存路径)

文件路径仅依赖于 lap_set.data_folder
'''
import sys
import os
import numpy as np 
import rospy
import math

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

joints_record_list = []     #机械臂7个关节角
lap_end_list = []           #根据夹具尺寸计算腹腔镜结构末端位置
lap_shaft_list = []         #前面3位为 points，后面3位 orientation
needle_position_list = []   #根据定位针尺寸计算末端位置

keyboard_monitor = key_signal.keyboard_monitor_class()

joint_velocity = 15/180*math.pi
        

        

rospy.init_node('rokae_init_pose')




joints = np.array([
    [0,     43.963,     0,      77.728,     6.349,      59.259,     79.145],    #对准下rcm
    [0,     67.568,     0,      75.524,     5.436,      -52.270,    81.927],    #对准前rcm
    [0,     34.276,     0,      48.969,     5.435,      48.827,     81.927]     #力传感器校准位姿
])
joints = joints * (np.pi/180)



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

def save_all_data():
    if len(needle_position_list) >0:
        needle_position_list.append(average_list(needle_position_list))
    save_data(joints_record_list, joints_record_path, '关节角位置')
    save_data(lap_end_list, lap_end_list_path, '腹腔镜末端位置')
    save_data(needle_position_list, needle_position_path, '标定针位置')
    save_data(lap_shaft_list, lap_shaft_path, "腹腔镜轴线")

rokae = rokae_basic_fun.rokae()

rokae.set_mode('jp')

tip_words = f'当前机械臂模式：{rokae.mode()}\n'\
        +'请选择初始化位姿：\n'\
        +'0: 对准下方rcm             \tj: 记录 关节角 \n'\
        +'1: 对准前方rcm             \tl: 记录 腹腔镜 末端位置\n'\
        +'2: 力传感器校准位姿         \to: 记录 腹腔镜 轴线\n'\
        +'d: 拖动模式（珞石自带）      \tn: 记录 定位针 位置\n'\
        +'f: 退出拖动模式（珞石自带）\n'\
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




