#!/usr/bin/env python3
#对rcm点和rcm坐标进行标定
import urx
import signal
import sys
import numpy as np

import select

import tty
import termios

import os

sys.path.append(f'{os.path.dirname(__file__)}/../src/optimal/scripts')
from lap_set_pk import lap_set


coordinate_set_path = lap_set.coordinate_set_path
camera_rcm_pose_file = lap_set.camera_rcm_pose_file
left_rcm_p_file = lap_set.left_rcm_p_file
right_rcm_p_file = lap_set.right_rcm_p_file




def write_file(file_path,data):
    if not os.path.exists(file_path):
        os.mknod(file_path)
    np.savetxt(file_path, data, delimiter=" ")
    # print(file_path.replace(coordinate_set_path,'')," writed")
    # print(file_path.replace(coordinate_set_path,'')," write: \n",data,'\n')
    


class keyboard_monitor_class(object):
    def __init__(self) -> None:
        signal.signal(signal.SIGINT, self.keyboard_interrupt)
        print("Press Ctrl+C to exit...")
        # 将终端设置为非规范模式,不修改的话需要回车，而且回车也会被读到
        self.orig_settings = termios.tcgetattr(sys.stdin)
        self.termios_resetted_flag = False
        tty.setcbreak(sys.stdin)
        pass

    def keyboard_interrupt(self,signal, frame):
        print("Keyboard Interrupt detected! (class)")
        # sys.exit(0) #用 raise 或者这个皆可
        raise KeyboardInterrupt 

    def detect(self):
        # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
        return select.select([sys.stdin], [], [], 0.1)

    def read_char(self):
        return sys.stdin.read(1)
    
    def monitor_stop(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
        self.termios_resetted_flag = True
        print("------恢复终端设置------")
        print("------键盘监听结束------")
        return 0
    
    def __del__(self):
        if not self.termios_resetted_flag:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
            print("------恢复终端设置------")


    




def main():
    
    keyboard_monitor = keyboard_monitor_class()

    rob = urx.Robot(lap_set.robot_ip)
    rob.set_tcp((0, 0, 0, 0, 0, 0)) 
    rob.set_payload(0.02, (0, 0, 0.05))

    tool_tip = lap_set.tool_tip
    T_rob_tool = lap_set.T_rob_tool
    print(f"T_rob_tool:\n{T_rob_tool}")



    try:
        print("请将拖动机械臂至定位针落于rcm点，然后按下对应键进行记录。\n\t左侧：l\n\t右侧：r\n\t腹腔镜：c\n")
        while True:
            # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
            rlist, _, _ = keyboard_monitor.detect()
            if rlist:
                # 读取单个字符并处理
                input_data = keyboard_monitor.read_char()
                

                if input_data == 'l':
                    print("左侧 rcm 点位置：")
                    rcm_l_point = np.squeeze(rob.get_pose().array @ tool_tip)
                    print(f"{rcm_l_point}\n")
                    write_file(left_rcm_p_file,rcm_l_point)


                elif input_data == 'r':
                    print("右侧 rcm 点位置：")
                    rcm_r_point = np.squeeze(rob.get_pose().array @ tool_tip)
                    print(f"{rcm_r_point}\n")
                    write_file(right_rcm_p_file,rcm_r_point)

                elif input_data == 'c':
                    print("腹腔镜 rcm 点位姿：")
                    rcm_c_pose = np.squeeze(rob.get_pose().array @ T_rob_tool )
                    print(f"{rcm_c_pose}\n")
                    write_file(camera_rcm_pose_file,rcm_c_pose)
                    
                
                else:
                    print("You typed:", input_data)
                    print("请将拖动机械臂至定位针落于rcm点，然后按下对应键进行记录。\n\t左侧：l\n\t右侧：r\n\t腹腔镜：c\n")


                    
            else:
                pass

            
            
    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!  (except)")
       

    # 恢复终端设置
    finally:
        keyboard_monitor.monitor_stop()
        rob.close()

        

if __name__ == '__main__':
    main()