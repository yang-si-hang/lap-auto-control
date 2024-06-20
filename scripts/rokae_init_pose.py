import sys
import os
import numpy as np 
import rospy

sys.path.append(f"{os.path.dirname(__file__)}")
import rokae_basic_fun 

sys.path.append(f"{os.path.dirname(__file__)}/my_tools")
import key_signal


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
    print(f'请选择初始化位姿：\n0: 对准下方rcm\n1: 对准前方rcm\n2: 力传感器校准位姿')
    while not rospy.is_shutdown():
        # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
        rlist, _, _ = keyboard_monitor.detect()
        if rlist:
            # 读取单个字符并处理
            input_data = keyboard_monitor.read_char()
            print(f'已选择位姿{input_data}')
            if input_data == '0':
                rokae.jp_cmd(joints[0])
            elif input_data == '1':
                rokae.jp_cmd(joints[1])
            elif input_data == '2':
                rokae.jp_cmd(joints[2])
            print(f'已抵达位姿{input_data}\n')
            print(f'请选择初始化位姿：\n0: 对准下方rcm\n1: 对准前方rcm\n2: 力传感器校准位姿\nctrl+c: 退出')

            # print("You typed:", input_data)

        else:
            input_data=''
            # count=0

        
        
# 程序中断处理        
except KeyboardInterrupt:
    print("Keyboard Interrupt detected!  (except)")
    rospy.signal_shutdown()
    
    

# 恢复终端设置
finally:
    keyboard_monitor.monitor_stop()




