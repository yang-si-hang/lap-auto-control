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

joints_record_list = []
lap_end_list = []  #根据夹具尺寸计算腹腔镜结构末端位置
needle_position_list = []

keyboard_monitor = key_signal.keyboard_monitor_class()

joint_velocity = 15/180*math.pi
        

        

rospy.init_node('rokae_init_pose')




joints = np.array([
    [0,     43.963,     0,      77.728,     6.349,      59.259,     79.145],    #对准下rcm
    [0,     67.568,     0,      75.524,     5.436,      -52.270,    81.927],    #对准前rcm
    [0,     34.276,     0,      48.969,     5.435,      48.827,     81.927]     #力传感器校准位姿
])
joints = joints * (np.pi/180)


rokae = rokae_basic_fun.rokae()

rokae.set_mode('jp')

tip_words = f'当前机械臂模式：{rokae.mode()}\n'\
        +'请选择初始化位姿：\n'\
        +'0: 对准下方rcm\n'\
        +'1: 对准前方rcm\n'\
        +'2: 力传感器校准位姿\n'\
        +'d: 拖动模式（珞石自带）\n'\
        +'f: 退出拖动模式（珞石自带）\n'\
        +'j: 记录当关节角 \n'\
        +'l: 记录腹腔镜末端位置\n'\
        +'n: 记录 定位针 位置\n'\
        +'ctrl+c: 退出'

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

            elif input_data == 'n':
                needle_position_list.append((rokae.pose.R_matrix()@lap_set.rob_needle_end_vector).squeeze()+rokae.pose.position.array())
                print(f'已记录 腹腔镜末端 位置：{len(needle_position_list)}：\n{needle_position_list[-1]}')
                print(rokae.pose.position)

            # 运动后重新显示提示词
            if input_data != 'j'\
                and input_data != 'l'\
                and input_data != 'n':
                print(f'已抵达位姿{input_data}\n')
                print(tip_words)


        else:
            input_data=''
            # count=0

# 程序中断处理        
except KeyboardInterrupt:
    print("Keyboard Interrupt detected!  (except)")
    # if rokae.mode=='drag':
    #     rokae.set_mode('jp')
    
    
    rospy.signal_shutdown()
    
# 恢复终端设置
finally:
    def save_data(data_list, file_path, key_word):
        '''
        将 data_list 转为矩阵并存储在 file_path
        并根据 key_word 显示保存数据名称
        Args:
            data_list (list): 需要保存的数据列表 
            file_path (str): 保存路径及文件名
            key_word (str)
        '''
        if len(data_list) != 0:
            np.savetxt(file_path, np.array(data_list), delimiter=',')
            print(f'已将 {len(data_list)} 个 {key_word} 储存在 {file_path}')
        else:
            print(f'未记录 {key_word} 信息 {len(data_list)}')

    save_data(joints_record_list, joints_record_path, '关节角位置')
    save_data(lap_end_list, lap_end_list_path, '腹腔镜末端位置')
    save_data(needle_position_list, needle_position_path, '标定针位置')

    keyboard_monitor.monitor_stop()




