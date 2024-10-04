'''
基于rokae_ros
选择以下几种机械臂位姿。
注意！！！ 只要在终端按下对应字母或序号，不用回车就会运行。
有两种：
1、进行关节角控制来运行到位姿
2、开启拖动模式，暂时不支持退出拖动模式（rokae_ros的问题）
'''
import sys
import os
import numpy as np 
import rospy

sys.path.append(f"{os.path.dirname(__file__)}")
import rokae_basic_fun 

sys.path.append(f"{os.path.dirname(__file__)}/my_tools")
import key_signal
joints_record_path = f"{os.path.dirname(__file__)}/../src/optimal/data/Camera_Calibration/rokae_lap_calibrate_joints.txt" #用于保存某些时刻的关节角
joints_record_list = []

keyboard_monitor = key_signal.keyboard_monitor_class()


        

        

rospy.init_node('rokae_init_pose')




joints = np.array([
    [0,     43.963,     0,      77.728,     6.349,      59.259,     79.145],    #对准下rcm
    [0,     67.568,     0,      75.524,     5.436,      -52.270,    81.927],    #对准前rcm
    [0,     34.276,     0,      48.969,     5.435,      48.827,     81.927]     #力传感器校准位姿
])
joints = joints * (np.pi/180)


rokae = rokae_basic_fun.rokae()

rokae.set_mode('jp')


try:
    print(f'当前机械臂模式：{rokae.mode()}\n请选择初始化位姿：\n0: 对准下方rcm\n1: 对准前方rcm\n2: 力传感器校准位姿\nd: 拖动模式（珞石自带）\nf: 退出拖动模式（珞石自带）\nr: 记录当前位姿')
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
                rokae.jp_cmd(joints[0])
            elif input_data == '1':
                rokae.jp_cmd(joints[1])
            elif input_data == '2':
                rokae.jp_cmd(joints[2])
            elif input_data == 'd' and rokae.mode != 'drag':
                rokae.set_mode('drag')
            elif input_data == 'f' and rokae.mode() == 'drag':
                rokae.set_mode('drag_stop')
                print('rokae_init_pose: drag_stop')
            elif input_data == 'r':
                joints_record_list.append(np.array(rokae.JointState.position).squeeze())
                print(f'已记录机械臂当前关节角{len(joints_record_list)}：\n{joints_record_list[-1]}')

            if input_data != 'r':
                print(f'已抵达位姿{input_data}\n')
                print(f'当前机械臂模式：{rokae.mode()}\n请选择初始化位姿：\n0: 对准下方rcm\n1: 对准前方rcm\n2: 力传感器校准位姿\nd: 拖动模式（珞石自带）\nf: 退出拖动模式（珞石自带）\nr: 记录当前位姿\nctrl+c: 退出')

                

            # print("You typed:", input_data)

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
    print(len(joints_record_list))
    if len(joints_record_list) != 0:
        np.savetxt(joints_record_path, np.array(joints_record_list), delimiter=',')
        print(f'已将 {len(joints_record_list)} 个关节角位置储存在 {joints_record_path}')
    else:
        print(f'未记录关节角信息 {len(joints_record_list)}')

    keyboard_monitor.monitor_stop()




