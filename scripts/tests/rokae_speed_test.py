"""
用于测试rokae的速度封装
"""
import numpy as np
import sys
import rospy
import random
import time

sys.path.append("/home/irobotcare/wyh/laparoscope_ws/scripts")
import rokae_basic_fun 


v_linear_max = 0.1
v_duration = 2 #s
record_rate = 100 #fps

def random_v_x(record_file_folder, rokae, v_num=10):
    """
    控制机器人以随机速度运动 v_num 次
    每次 v_duration 秒
    并持续以 record_rate fps 记录速度指令以及当前速度

    Args:
        record_file_folder (str): 数据保存的文件夹路径
        rokae (rokae_basic_fun.rokae): 封装的rokae类对象
        v_num (int, optional): 随机运动次数 Defaults to 10.
    """
    global v_linear_max, v_duration, record_rate

    v_cmd_file = f"{record_file_folder}/v_cmd.txt"
    v_now_file = f"{record_file_folder}/v_now.txt"
    v_cmd = [0,0,0,0,0,0]
    v_now = [0,0,0,0,0,0]

    v_cmd_record_list = []
    v_now_record_list = []
    for i in range(2*record_rate):
        v_cmd_record_list.append([time.perf_counter()]+v_cmd)
        v_now_record_list.append([time.perf_counter()]+v_now)
        time.sleep(1/record_rate)

    for i in range(int(v_num/2)):
        v_cmd[0] = random.uniform(-v_linear_max, v_linear_max)  #只随机x方向运动
        print(f"i: cmd = {v_cmd}")
        rokae.cv_cmd(v_cmd)
        v_cmd_record_list.append([time.perf_counter()]+v_cmd)
        for i in range(v_duration * record_rate):
            v_now = rokae.cv_now
            v_now_record_list.append([time.perf_counter()]+v_now)
            v_cmd_record_list.append([time.perf_counter()]+v_cmd)
            time.sleep(1/record_rate)

        v_cmd[0] = -v_cmd[0]
        rokae.cv_cmd(v_cmd)
        v_cmd_record_list.append([time.perf_counter()]+v_cmd)
        for i in range(v_duration * record_rate):
            v_now = rokae.cv_now
            v_now_record_list.append([time.perf_counter()]+v_now)
            v_cmd_record_list.append([time.perf_counter()]+v_cmd)
            time.sleep(1/record_rate)
    rokae.stop()
    
    np.savetxt(v_cmd_file, np.array(v_cmd_record_list, dtype=float), delimiter=',')
    np.savetxt(v_now_file, np.array(v_now_record_list, dtype=float), delimiter=',')


if __name__ =="__main__":
    rospy.init_node("rokae_speed_test")
    rokae = rokae_basic_fun.rokae()
    rokae.set_mode('cv')
    random_v_x(f"/home/irobotcare/桌面/EX_Data/其他测试/rokae_v", rokae, 10)

    


