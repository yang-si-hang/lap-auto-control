import time
import os,sys
from datetime import datetime
import numpy as np


sys.path.append(f"{os.path.dirname(__file__)}")
import key_signal

data_folder = "/home/irobotcare/桌面/EX_Data/其他测试/drag"
now = datetime.now()
# 格式化时间为 'YYYY-MM-DD HH:MM:SS' 格式
formatted_time = now.strftime("%Y%m%d%H%M%S")
data_file = f"{data_folder}/操作时长_{formatted_time}.txt"

keyboard_monitor = key_signal.keyboard_monitor_class()

operating = False
start_time = 0
finish_time = 0
operate_time_list = []


if __name__ == "__main__":

    try:
        print(f"请输入 回车 开始计时")
        while True:
            # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
            rlist, _, _ = keyboard_monitor.detect()
            if rlist:
                # 读取单个字符并处理
                input_data = keyboard_monitor.read_char()
                # print("You typed:", input_data)
                if input_data =="\n":
                    if operating:
                        finish_time = time.perf_counter()
                        operate_time_list.append(finish_time-start_time)
                        operating = False
                        print(f"第{len(operate_time_list)}次 计时结束 用时：{operate_time_list[-1]} s")
                        print(f"==================================")
                    else:
                        start_time = time.perf_counter()
                        operating = True
                        print(f"开始计时...")

            else:
                input_data=''
                # count=0
            
    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!  (except)")
       

    # 恢复终端设置
    finally:
        keyboard_monitor.monitor_stop()
        np.savetxt(data_file, operate_time_list, delimiter=",")