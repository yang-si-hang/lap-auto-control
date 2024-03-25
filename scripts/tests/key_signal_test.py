#!/usr/bin/env python3
#这种方法只监听该终端内键盘的输入

import signal
import sys
import select
import tty
import termios

class keyboard_monitor_class(object):
    def __init__(self) -> None:
        signal.signal(signal.SIGINT, self.keyboard_interrupt)
        print("Press Ctrl+C to exit...")
        print("键盘监听开始...")
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

    try:
        while True:
            # 检查标准输入是否有可读数据,没有这段好像也不影响ctrl c 中断
            rlist, _, _ = keyboard_monitor.detect()
            if rlist:
                # 读取单个字符并处理
                input_data = keyboard_monitor.read_char()
                print("You typed:", input_data)

            else:
                input_data=''
                # count=0

            
            
    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!  (except)")
       

    # 恢复终端设置
    finally:
        keyboard_monitor.monitor_stop()
        

        

if __name__ == '__main__':
    main()