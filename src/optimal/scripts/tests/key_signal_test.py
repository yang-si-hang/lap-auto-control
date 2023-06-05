#!/usr/bin/env python3
#这种方法只监听该终端内键盘的输入

import signal
import sys

import select

import tty
import termios

def main():
    

    def keyboard_interrupt(signal, frame):
        print("Keyboard Interrupt detected!")
        # sys.exit(0) #用 raise 或者这个皆可
        raise KeyboardInterrupt 

    signal.signal(signal.SIGINT, keyboard_interrupt)

    print("Press Ctrl+C to exit...")

    # 将终端设置为非规范模式,不修改的话需要回车，而且回车也会被读到
    orig_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin)

    try:
        while True:
            # 检查标准输入是否有可读数据
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                # 读取单个字符并处理
                input_data = sys.stdin.read(1)
                print("You typed:", input_data)

            else:
                input_data=''
                # count=0

            
            
    # 程序中断处理        
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected!")
       

    # 恢复终端设置
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("------恢复终端设置-----")
        

if __name__ == '__main__':
    main()